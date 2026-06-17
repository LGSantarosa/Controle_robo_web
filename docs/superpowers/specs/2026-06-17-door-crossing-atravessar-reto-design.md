# door_crossing — prioridade = ATRAVESSAR reto (A+B+C)

**Data:** 2026-06-17
**Estado base:** `a40c211` (door_crossing já deployado+buildado na Pi)
**Escopo:** 3 patches pequenos e reversíveis EM CIMA da FSM atual
(`idle→staging→rotating→crossing`). NÃO reescreve a máquina de estados — ela
atravessou a porta em campo (06-12) e fica intacta na estrutura. Muda só o
**critério de "pronto pra cruzar"** e o que acontece nas bordas (re-armar).

## Problema (observado em campo 2026-06-17)

O objetivo embutido na FSM é "ficar alinhado", não "atravessar". Dois sintomas:

1. **Veio reto, mas ficou caçando o meio.** A pipeline é obrigatória e
   sequencial: mesmo chegando reto e dentro do vão, o robô é forçado a (a) dirigir
   até o ponto de staging exato (`stage_tol=0.10`), (b) segurar `|lat|<8cm` E
   `|yaw|<5°` por 5 ticks, (c) e o `rotating` volta pro `staging` toda vez que o
   lateral passa de 8cm — como o skid-steer arrasta ao girar, ele estoura, volta,
   re-stage = **ping-pong "caçando o meio"**. Tolerâncias apertadíssimas (8cm) num
   vão de ~93cm onde o robô (~50cm) tem ~21cm de folga de cada lado.

2. **Atravessou e voltou a buscar a porta.** Ao cruzar, vai pra `idle` SEM
   cooldown. O `/plan` é publicado a ~1Hz → por ~1s depois de sair, o plano ainda
   mostra a rota velha cruzando a porta → re-arma → recalcula o `side` → o lado de
   aproximação INVERTE → tenta atravessar de volta.

## Restrição de projeto (do usuário)

Depois da porta há um **corredor só um pouco mais largo que ela**. Se o robô sair
**torto**, vai de cara na parede. Logo:

- O **ângulo continua apertado** (~5°): a garantia do "reto". Quem deixa o robô
  reto é o **giro no lugar (point-turn) do `rotating`**, NUNCA um arco dentro do
  vão (⛔ skid-steer não faz arco — lição do projeto).
- A **posição lateral é que afrouxa** ("não precisa estar no x exato do meio").
- Mas afrouxar a posição não pode deixar ele entrar torto/de lado e raspar o
  batente → o critério tem que ser **"dá pra ir reto sem bater?"**, não só ângulo.

## Solução — A + B + C

### A. Checagem universal "passo reto daqui?" → ATRAVESSA (todo tick)

A ideia do usuário NÃO é um atalho só no momento de armar — é a pergunta
**`passo reto daqui?` rodando a TODO momento** do door_crossing. No instante em
que o robô fica reto e cabendo (em qualquer fase: `staging` OU `rotating`), ele
**atravessa na hora**, sem esperar ticks estáveis. Isso ataca o sintoma exato:
"ele alinhava e voltava a CAÇAR o meio" (re-staging/re-alinhar) em vez de só
passar.

Implementação: uma checagem única no topo do bloco de estado ativo (depois das
guardas de segurança e do abort por `gap < gap_min`), antes da lógica de
`staging`/`rotating`:

```
se pode_ir_reto:  state = 'crossing'; return cmd de crossing
```

Como o armar cai em `staging` e segue pro mesmo tick, isto também cobre o "veio
reto, passa" (atravessa já no tick de armar). Vale de qualquer distância na
`zone_radius`. O `align_stable` (esperar N ticks alinhado) **deixa de existir** na
decisão — era justamente o "alinhou mas continua verificando".

### B. Critério único `pode_ir_reto` + fim do ping-pong

O critério de "pronto pra cruzar" passa a ser, no atalho (A) E na transição
`rotating→crossing`:

```
fit_lat   = max(0, geom.half_width − robot_half_width − fit_margin)
pode_ir_reto  ⇔  |yaw_err| ≤ align_yaw   E   |d| ≤ fit_lat
```

`fit_lat` é a **folga geométrica real**: "dá pra passar reto sem encostar nos
batentes?", derivada da largura da porta MARCADA pelo usuário (`geom.half_width`,
que já existe) e da meia-largura do robô. Auto-ajusta:

- Porta larga (~0.93m): `fit_lat ≈ 0.16m` (relaxa, não precisa do meio exato).
- Porta apertada (~0.70m): `fit_lat ≈ 0.05m` (exige bem mais centro, mal cabe).

(Conta: `0.70/2 − 0.25 − 0.05 = 0.05`. Robô de ~0.50m num vão de 0.70m tem só
~10cm de folga TOTAL → `fit_lat` apertado é correto e desejado.)

Efeitos no `rotating`:
- A transição pro `crossing` saiu do `rotating` (virou a checagem universal A).
  O `rotating` agora só: gira no lugar pra endireitar o yaw, e se está
  genuinamente FORA do vão (`|d| > fit_lat`) volta pro `staging` reaproximar.
- A volta-pro-`staging` usa `fit_lat` (em vez do 8cm fixo) → só reaproxima do
  eixo quando genuinamente NÃO cabe reto → acaba o ping-pong.
- `align_yaw` continua 5° (apertado = reto). O `rotating` point-turn endireita no
  lugar; ninguém arca dentro do vão.

O `crossing` mantém a micro-correção que já tem (`cross_k_lat`/`cross_k_yaw`,
teto `cross_wz_max`): entrando já reto e dentro de `fit_lat`, a correção é
pequena (e a lateral corrige PRA o centro = pra dentro de mais folga). A
centralização fina acontece em movimento, não parado.

### C. Trava pós-travessia (cooldown 2s)

Na saída limpa do `crossing` (`s > exit_margin`), antes de ir pra `idle`, setar
`self._cooldown_until = now + success_cooldown` (`success_cooldown = 2.0s`). Cobre
a janela do `/plan` defasado: em ~1s o plano atualiza e o robô já saiu →
`_pick_door` não re-arma → o `side` não inverte → não tenta voltar.

## Parâmetros (todos live-tunáveis via callback `_on_set_params`)

| param | hoje | novo | papel |
|---|---|---|---|
| `align_yaw_deg` | 5.0 | 5.0 (mantém) | ângulo apertado = garantia do reto |
| `align_lat` | 0.08 | — (deixa de ser o gate; vira `fit_lat` geométrico) | — |
| `align_stable` | 5 | **DEPRECATED** | a checagem universal cruza na hora, sem esperar ticks |
| `robot_half_width` | — | **0.25** (novo) | meia-largura do robô p/ `fit_lat` |
| `fit_margin` | — | **0.05** (novo) | folga de segurança subtraída no `fit_lat` |
| `success_cooldown` | — | **2.0** (novo) | trava pós-travessia (C) |

`align_lat` continua **declarado** como param (compat — não quebra quem já seta),
mas **deixa de ser o gate** de "pronto pra cruzar": esse papel passa pro `fit_lat`
geométrico. `align_lat` não entra mais na decisão de readiness nem na volta-pro-
staging.

## Refino pós-campo (2026-06-17, 2ª rodada)

Validação em campo do A+B+C: "passou 4× liso, confiante e rápido", melhor que
antes. Mas dois ajustes ("diminuir um pouco a confiança"):

1. **Bateu no batente uma vez:** girando rápido (`rot_speed=4.0`≈11,5°/tick), num
   tick o yaw caiu na banda de ±`align_yaw` → ativou `crossing` → a **inércia
   angular** levou o robô torto pra dentro → bateu. A checagem olhava o ângulo
   INSTANTÂNEO, sem ver se ele tinha PARADO de girar.
   **Fix:** o "passo reto daqui?" só ativa o crossing quando, além de reto+cabe, a
   **taxa de giro real ≈ 0** (`cross_yaw_rate_max`, default 0,5 rad/s, medida de
   dois ticks de pose). "Alinhou E parou, aí passa." Vindo reto do nav2 (não está
   girando) a taxa já é ~0 → cruza na hora; no meio de um giro rápido → espera
   assentar. O `rotating` passa a COMANDAR PARAR (wz=0) quando entra na banda de
   ±`align_yaw` (não gira mais; deixa assentar), em vez de dar mais um giro.
   *Resíduo a observar em campo:* overshoot além de ±`align_yaw` re-corrige a
   `rot_speed` cheia (não baixar, stalla); no skid-steer (atrito alto) deve
   convergir, mas se "tremer" na porta sem cruzar, adicionar banda de
   amortecimento.
2. **Ativava demais o "indo pro eixo" (staging) já estando no meio:** quando já
   centrado (`|d|≤fit_lat`) mas com yaw torto, o `staging` perseguia o PONTO exato
   de preparação (às vezes recuando) antes de alinhar.
   **Fix:** já no eixo → vai DIRETO pro `rotating` (alinha no lugar), sem perseguir
   o ponto. O `staging` passa a servir só pra quando está FORA do eixo
   (`|d|>fit_lat`) — aí dirige PRO eixo, e ao entrar no fit vira `rotating` em
   qualquer distância. Removido o `stage_tol` (chegar no ponto) como gatilho.

Novo param live-tunável: `cross_yaw_rate_max` (0,5). Knobs de campo agora:
`fit_margin` (raspou?), `cross_yaw_rate_max` (cruzou cedo demais girando? baixa;
demora a cruzar? sobe), `success_cooldown`.

## Refino pós-campo (2026-06-17, 3ª rodada) — FREIO no giro

A 2ª rodada bateu no batente DE NOVO. Log (`door_crossing:` transições) mostrou
~11s de briga `staging↔rotating↔reversing` antes de cruzar torto: o robô "girou
esquerda-direita rápido demais". **Causa raiz (confirmada):** o point-turn a
`rot_speed=4.0` = **11,5°/tick** PASSA DIRETO da banda de ±`align_yaw` (10° de
largura) → na outra ponta recomeça giro a velocidade cheia → **oscila** sem
assentar. A trava de taxa (2ª rodada) impediu de cruzar no meio do giro, mas não
curou a oscilação. (A ré que disparou no meio estava CORRETA — ia de cara no
batente, recuou; é a proteção anti-batida, mantida.)

**Fix — freio nos últimos graus:** o point-turn usa velocidade cheia quando
`|yaw_err| > rot_brake_angle` (12°, quebra o atrito, gira rápido) e desacelera pra
`rot_brake_speed` (2,0 rad/s ≈ 5°/tick) dentro disso → ENCAIXA na banda sem
overshoot → assenta → cruza reto. NÃO é o "proporcional" reprovado (que abaixava o
giro inteiro): aqui só os últimos ~7° (12°→5°), e o robô já está girando (atrito
quebrado) → não stalla. Sem boost no freio. Novos params live: `rot_brake_deg`
(12), `rot_brake_speed` (2,0).

**+ Log de campo por tick:** a FSM expõe `dbg_yaw_err/dbg_d/dbg_yaw_rate`; o nó
loga na transição E estrangulado (~2 Hz) durante a manobra (`yaw_err° lat_cm taxa
vx wz gap front`) — pra CONFIRMAR no próximo teste em vez de adivinhar.

## Não-objetivos (YAGNI)

- NÃO reescrever a FSM nem colapsar estados (era a opção "D", recusada).
- NÃO mexer no `crossing` além de reusar a correção existente.
- NÃO afrouxar o yaw (corredor estreito pós-porta exige reto).
- NÃO tocar no collision monitor, scan_sanitizer, unstuck standdown, arming pelo
  `/plan` (`plan_crosses_door`) nem na ré de escape — tudo fica como está.

## Testes

Lógica pura → tudo offline (`robot_nav/test/test_door_crossing.py`).

Ajustar dos 43 existentes os que cravam o 8cm/5-ticks/ping-pong. Novos:
- **Atalho positivo:** armar já reto + dentro de `fit_lat` → `crossing` direto
  (não passa por `staging`).
- **Atalho negativo (lateral):** armar reto mas `|d| > fit_lat` → `staging`.
- **Atalho negativo (yaw):** armar centrado mas yaw fora → `staging`/`rotating`.
- **`fit_lat` geométrico:** porta larga aceita `|d|` maior; porta apertada exige
  `|d|` menor (mesma pose, portas de larguras diferentes → decisões diferentes).
- **Sem ping-pong:** no `rotating`, drift lateral pequeno (dentro de `fit_lat`)
  NÃO volta pro `staging`.
- **Cooldown (C):** logo após saída limpa do `crossing`, re-arme é suprimido por
  `success_cooldown`; após ele, arma normal.

## Deploy / validação (robô estava DESLIGADO no design)

Tudo é Python puro, sem reflash. Deploy = `git fetch && reset --hard origin/main`
+ `colcon build --packages-select robot_nav` na Pi (`~/workspace/Controle_robo_web`,
`ssh robo@robo-desktop.local`). Usuário relança a stack e valida em campo:
(a) chegando reto → atravessa direto, sem caçar o meio; (b) não volta a buscar a
porta depois de passar; (c) sai reto no corredor estreito sem raspar. Knobs de
campo: `fit_margin`, `align_stable`, `success_cooldown` (live).

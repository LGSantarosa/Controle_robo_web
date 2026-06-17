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

### A. Atalho "tá reto e cabe → ATRAVESSA"

No armar (`idle`), depois de escolher a porta e o `side`, calcular `d` (offset
lateral) e `yaw_err` ali mesmo. Se `pode_ir_reto` (def. abaixo) → vai **direto
pra `crossing`**, pulando `staging` e `rotating`. Senão → `staging` (fluxo de
hoje). Vale de **qualquer distância** dentro da `zone_radius` (decisão do
usuário: "veio reto, passa").

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
- "Alinhado" = `pode_ir_reto`, segurado por `align_stable=2` ticks (era 5).
- A volta-pro-`staging` passa a usar `fit_lat` (em vez do 8cm fixo) → só
  reaproxima do eixo quando genuinamente NÃO cabe reto → acaba o ping-pong.
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
| `align_stable` | 5 | **2** | commita mais rápido quando reto+cabe |
| `robot_half_width` | — | **0.25** (novo) | meia-largura do robô p/ `fit_lat` |
| `fit_margin` | — | **0.05** (novo) | folga de segurança subtraída no `fit_lat` |
| `success_cooldown` | — | **2.0** (novo) | trava pós-travessia (C) |

`align_lat` continua **declarado** como param (compat — não quebra quem já seta),
mas **deixa de ser o gate** de "pronto pra cruzar": esse papel passa pro `fit_lat`
geométrico. `align_lat` não entra mais na decisão de readiness nem na volta-pro-
staging.

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

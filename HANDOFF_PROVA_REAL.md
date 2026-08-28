# HANDOFF — PROVA REAL do nav2_trekking (a partir de 2026-08-27)

> **⚠️ 2026-08-28 — o pacote `nav2_trekking` NÃO EXISTE MAIS.** Ele foi dissolvido de
> volta no `robot_nav` (branch `arena-galpao`): a geometria virou o perfil
> `config/nav2_params_arena.yaml`, selecionável por `./launch.sh --nav2 --arena`.
> O fork nunca rodou pelo `launch.sh` nem no robô. Ver
> `docs/superpowers/specs/2026-08-28-arena-galpao-design.md`.

> **Para o Claude que pegar esta sessão no robô.** Leia isto ANTES de tocar em
> qualquer coisa. O dono chega no robô, liga, e quer rodar a prova.
>
> Ordem dele, literal:
> *"o que conquistamos agora irá ser testado no robô, sem mais mexer nele, vamos
> tirar a real prova antes de mandar um monte de modificação e ele explodir sem
> sabermos o que foi"*
>
> **Logo: NÃO mexa em parâmetro nenhum antes da prova.** Sua função aqui é
> deployar, rodar, medir e diagnosticar — não melhorar. A fila de melhorias já
> existe e está no fim deste arquivo; ela espera.

---

## 1. O que exatamente vai ser provado (é UMA coisa só)

O `nav2_trekking` passou a declarar no costmap **`robot_radius: 0.32`** em vez do
footprint quadrado ±0.25, e a inflação subiu (global 0.45→**0.60**, local
0.35→**0.45**). Commit **`e03555b`**, branch **`nav2-trekking`**.

**Por quê:** o robô é um QUADRADO de 0,5 m e tem dois raios — **inscrito 25 cm**
(quando alinhado com a parede) e **circunscrito 35,4 cm** (o canto, quando anda
em diagonal). O nav2 usa o INSCRITO como fronteira do proibido, então o planner
traçava rotas legais para um círculo de 25 cm enquanto o corpo real ocupava 35,4.
Medido nos 4 contatos de uma volta: o robô estava **sempre em diagonal**, centro
a 29,6-35,5 cm da parede, corpo alcançando 34,8-35,3 cm. **Margem zero por
construção** — e os corredores ali tinham 1,56-1,87 m: sobrava mais de um metro,
ele passava colado porque nada no stack proibia.

**Motivo do dono (é o critério de aceitação):**
*"eu n quero que ele bata mano, isso é ruim demais, pq na vida real ele pode
errar na pose e bater de verdade, o gazebo tem que estar melhor"*.
A folga existe pra ser o colchão do erro de AMCL + derrapagem do skid. É por isso
que a prova real importa: **no sim o erro de pose é zero, no real não é** — a
prova é justamente ver se 3-6 cm de folga bastam quando a pose erra.

### Resultado no sim, pra você ter contra o que comparar

| | **baseline (nav2 padrão, `robot_nav`)** | **o que vai ser provado** |
|---|---|---|
| goals | 7 de 8 | **16/16** (2 voltas) |
| tempo | 791,6 s | **654,3 s** (650 / 658) |
| v média | 0,177 m/s | **0,233 m/s** |
| colisões / raspões | — | **0 / 0** |
| folga mínima real | — | **3,7 cm** (antes: 0,1 cm) |

⚠️ **"baseline" neste projeto = o nav2 PADRÃO (`robot_nav`), 791,6 s, 7/8.** Não
use a palavra pra nenhum degrau intermediário nosso — misturar os dois já
inverteu o sinal de um relatório inteiro e confundiu o dono.

⚠️ **Compare v média (dist/tempo), não relógio bruto.** O planner sorteia rota
diferente a cada volta: entre as runs a distância variou de 140,1 a 156,5 m (11%).
Relógio bruto é contaminado pela rota sorteada.

---

## 2. Deploy (robô DESLIGADO ainda dá; a Pi precisa estar ligada e na rede)

O pacote **`nav2_trekking` nunca existiu na Pi** — é a primeira vez que sobe.

1. A branch `nav2-trekking` **não está na `main`**, e a Pi normalmente faz
   `git reset --hard origin/main`. **Pergunte ao dono** se prefere push da branch
   (e checkout dela na Pi) ou merge na main. Não decida sozinho.
2. Na Pi: **`git fetch` + `git reset --hard <ref>`**. **NUNCA `scp`** — o repo da
   Pi é `~/workspace/Controle_robo_web`.
3. Na Pi: **`colcon build --packages-select nav2_trekking`** (é pacote novo lá;
   `wheel_msgs` é compartilhado com o `robot_nav` e já existe).
4. SSH na Pi: `ssh robo@robo-desktop.local`, e **use retry** —
   `until ssh ...; do sleep 3; done`. Pi offline = bateria do robô OU o PC em
   WiFi diferente; não especule crash.

---

## 3. ⚠️ ANTES DE RODAR: confira o mapa novo

**A prova vai ser em outro lugar, com outros mapas** (informação do dono). Isso
importa muito por causa de um efeito colateral do fix:

> **O nav2 trata tudo abaixo do `robot_radius` como BLOQUEIO.** A fresta mínima
> que o robô atravessa subiu de **~0,50 m para ~0,64 m**.

Se o lugar novo tiver uma porta ou corredor estreito, o robô **simplesmente não
acha caminho** — e isso parece bug misterioso se você não souber. Rode isto assim
que o mapa novo estiver salvo (é só análise de imagem, não sobe nada):

```bash
python3 tools/mapa_passagens.py maps/<mapa_novo>.yaml
```

Ele diz se o mapa continua inteiro com o robô de 32 cm e aponta as coordenadas
das passagens mais apertadas. Referência dos mapas conhecidos:

| mapa | raio 0,25 | raio 0,32 |
|---|---|---|
| `sala` (golden) | 100% inteiro | **100% inteiro** (gargalos de 0,64 m — no limite) |
| `sala_grande` (sim) | 100% | **100%** |
| `hotmilk` | 89,2% | 82,0% (**perde 7%**) |

Se o mapa novo perder pedaço grande: **avise o dono antes de rodar** e ofereça
baixar o raio (0,30 → fresta de 0,60 m). Não baixe por conta própria — a prova é
justamente do 0,32.

---

## 4. Como medir no real (não existe ground truth aqui)

No sim eu tinha o `tools/sim_ab/colisao.py`, que lê a pose verdadeira do Gazebo e
calcula a folga em milímetros contra a geometria do mundo. **No robô real isso não
existe.** O que existe:

- **O olho do dono.** Encostou ou não encostou é observação dele. É o dado
  primário — não invente precisão que você não tem.
- **`min_scan` do `/scan`** — o laser fica no CENTRO do robô, então a leitura é
  distância do centro. **É PROXY, não segurança**: 0,25 m tanto pode ser "passei
  raspando" quanto "encostei". Já errei feio tratando `min_scan` como se fosse
  folga; não repita.
- **CSVs que os nós gravam sozinhos** em `controle_web/logs/` (eles SOBRESCREVEM
  a cada launch — **arquive logo depois da run**): `follow_debug.csv` (estado do
  seguidor, `vx`, `wz`, `clear`), `unstuck.csv`, `freeze_capture.csv`.
- **O log do nav2** — é onde os sintomas da seção 5 aparecem.

**Instrumente em CSV na Pi que VOCÊ puxa por ssh.** Nunca peça pro dono ler
console e relatar: ele roda, você lê e diagnostica.

---

## 5. 🔎 Tabela de diagnóstico — sintoma → suspeito

Isto é o coração deste handoff. Se algo der errado, comece por aqui.

| sintoma | suspeito | por quê / o que fazer |
|---|---|---|
| `"Could not generate path between the given poses"` | **O `robot_radius` 0.32 fechou uma passagem** | É O efeito colateral conhecido do fix. Rode `mapa_passagens.py` no mapa. Foi assim que o raio 0.354 foi reprovado (fechou vãos de 0,70 m). |
| `"Either of the start or goal pose are an obstacle!"` | **BUG ABERTO, NÃO é o fix** | Já acontecia ANTES e PIOR (16 numa volta com a config velha, 5 com a nova). Medi o robô a **42-72 cm** de parede quando isso apareceu — longe do raio (0,32) E da inflação (custo ~93 ali, contra 253 de letal). Suspeita: `obstacle_layer` marcando fantasma ou não limpando marcação velha. **É quem gera os recoveries.** Não debite isso da conta do fix. |
| robô encosta na parede mesmo assim | erro de AMCL maior que a folga | É EXATAMENTE o que a prova quer descobrir. Meça quanto: a folga projetada é 3-6 cm. Se o AMCL erra 10 cm, o fix é insuficiente e o caminho é subir o raio (ou a inflação), não abandonar. |
| gira parado muito tempo / "minicurvinhas" | **conhecido, não é regressão** | 34% da volta é giro parado e 72-87% do giro se cancela. QUATRO tentativas no seguidor já falharam (`turn_enter`, `aim_tau`, `lookahead`, `rot_min`). A causa é o plano quebrado do Theta*. Está na fila, não ataque agora. |
| anda devagar (0,25-0,40 em vez de 0,60) | `speed_for_clearance` | Conhecido: 28,6% do tempo de condução é rastejo por causa desse freio. **Está na fila como próximo passo #1.** Não mexa antes da prova. |
| robô "gira sozinho" / não obedece | **processo órfão** | Confira `ps` ANTES de culpar código. Launch duplo, `app.py` órfão, `teleop_twist_joy` órfão (prio 100 no twist_mux, sequestra o robô), `parameter_bridge` órfão (2 fontes de `/clock`). Já me queimei culpando o atuador quando era script meu rodando. |
| nav2 sobe e o robô fica parado sem publicar nada | bringup travado no lifecycle | Cheque `ros2 lifecycle get /bt_navigator` ANTES de suspeitar do robô. |

---

## 6. O que NÃO mexer (e o que vem DEPOIS da prova)

**Congelado até a prova terminar.** Tudo abaixo já foi medido; mexer agora
adiciona suspeito e é exatamente o que o dono pediu pra não fazer.

Já testado e **reprovado** (não re-tentar): `lookahead` 1.0 (11 colisões, e o
"ganho" era atalho colado na parede) · `aim_tau` 1.6 · `turn_enter` 24° ·
`rot_min` 3.0 (rendeu 2%, não paga derrapagem no skid real) · `robot_radius`
0.354 (quebra o planner) · modo `approach` do collision_monitor (deadlock do
point-turn) · caixa fixa `stop` · baixar inflação junto com o reflexo.

**Fila pós-prova**, em ordem (detalhes em `HANDOFF_NAV2_TREKKING.md` §6):
1. Afrouxar o `speed_for_clearance` — é onde está o tempo. Aquele freio existia
   pra compensar o robô passar colado; agora o plano garante folga, então ele
   cobra duas vezes pela mesma segurança.
2. O bug do `"start pose is an obstacle"` (é quem gera recovery).
3. Suavizar o plano (`nav2_smoother` fora do launch, BT sem `SmoothPath`).

---

## 7. Higiene (aprendida à força)

- **Robô LIGADO** pra prova. Avise o dono quando precisa ligado vs desligado.
- **Anuncie e ESPERE o "pode"** antes de abrir qualquer janela de captura ou
  mandar o robô andar.
- **Pare tudo no fim** — não deixe nó rodando dirigindo o robô.
- **Uma mudança por vez.** Se a prova pedir ajuste, ajuste UMA coisa e meça.

Documentos irmãos: **`HANDOFF_NAV2_TREKKING.md`** (o histórico técnico completo:
o que foi testado, o que falhou e por quê) e `ESTADO_PROJETO.md`.

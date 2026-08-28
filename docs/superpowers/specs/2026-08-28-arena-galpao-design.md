# Design — Arena do galpão (prova de 2026-09-05)

> Sessão de 2026-08-28. Substitui o objetivo da fase anterior (`HANDOFF_PROVA_REAL.md`),
> que era provar velocidade e margem. **A meta mudou: nada de velocidade — o robô
> tem que ser destemido no movimento e MUITO preciso.**

---

## 1. A missão (palavras do dono)

Numa arena de galpão:

```
LARGADA → cone 1 → cone 2 → cone 3 → cone 4 → CHEGADA
```

- Cada cone fica em cima de uma **plataforma amarela grande** no chão.
- O **cone é a âncora de detecção** (o LiDAR o enxerga); a **plataforma é o alvo**.
- **Chegar a 20 cm do cone = já está na plataforma = ponto marcado.**
- Ao marcar o ponto, **um LED acende** (relé a ser remontado no robô).
- Entre os pontos há **blocos com frestas de 60 / 70 / 80 / 90 cm**.
  **Passar pela fresta é OPCIONAL** — é atalho, ganha tempo. Contornar sempre é possível.
- **Sem limite de tempo.** O critério é chegar em todos os pontos **sem bater em nada**.

---

## 2. Critérios de aceitação

| # | critério | como se mede |
|---|---|---|
| A1 | Visita os 4 cones + chegada, na ordem | log da missão |
| A2 | Para a ≤ 20 cm do cone (borda do robô → superfície do cone) | pose vs cone no sim; trena no real |
| A3 | LED acende em cada ponto marcado | observação + CSV |
| A4 | **Zero contato** com bloco, cone ou parede | `tools/sim_ab/colisao.py` no sim; olho do dono no real |
| A5 | Completa a missão mesmo **sem** usar nenhuma fresta | rodar com o atalho desligado |

**A5 é a rede de segurança do prazo:** se o módulo de fresta não ficar pronto, a missão
ainda fecha. O risco fica isolado no bônus, não na entrega.

---

## 3. Geometria — números confirmados nesta sessão

| grandeza | valor | fonte |
|---|---|---|
| Largura total do robô (pneu a pneu) | **0,50 m** | trena do dono, 2026-08-28 |
| Corpo (carcaça) | 0,50 × 0,50 × 0,26 m | `robot.urdf.xacro:19-21` |
| Raio inscrito / circunscrito | 0,25 / **0,354** | geometria do quadrado |
| Altura do LiDAR (LD20) | 0,465 m do chão | `robot.urdf.xacro:114` + base_link a 0,215 |
| Cone real | mais alto que o LiDAR | dono |

### 🐛 Bug encontrado: `track_width` da URDF real está errado

`robot.urdf.xacro:17` declara `track_width = 0.50` como **centro-a-centro** das rodas.
Com `wheel_width = 0.06`, a borda externa cai em 0,25 + 0,03 = **0,28 → 0,56 m de largura**.
A trena diz **0,50 m total**. O valor correto é **`track_width = 0.44`**.

O `sim_robot.sdf` já estava certo (rodas em ±0,22 → borda em 0,25).

**Impacto:** a URDF alimenta o TF e o raciocínio de geometria — não o `robot_radius` do
costmap, que é parâmetro à parte. Mas 6 cm de largura fantasma é a diferença entre
5 cm e 2 cm de folga numa fresta de 60. **Fix é o passo 1.**

### A conta da fresta de 60 cm

Robô 0,50 m numa fresta de 0,60 m: **5 cm por lado, e só se entrar reto.**

Largura efetiva com yaw θ: `W(θ) = 0,50·(cos θ + sen θ)`

| yaw | largura efetiva | folga lateral restante |
|---|---|---|
| 0° | 0,500 m | ±5,0 cm |
| 5° | 0,542 m | ±2,9 cm |
| 10° | 0,579 m | ±1,0 cm |
| **13°** | **0,600 m** | **0 — encosta** |

**Orçamento de erro na fresta de 60: yaw ≤ 5° E lateral ≤ ±3 cm.**
O `door_crossing` hoje fecha em `|lat| < 8 cm, |yaw| < 5°` — o yaw já serve, o lateral
precisa apertar para ~3 cm.

---

## 4. Arquitetura — dois níveis que degradam sozinhos

### Nível 1 — nav2 como está hoje (`robot_radius: 0.32`) = MODO SEGURO

O `0.32` exige vão ≥ 0,64 m. Consequência, sem mexer em nada:

| fresta | nav2 puro |
|---|---|
| 90 cm | ✅ passa |
| 80 cm | ✅ passa |
| 70 cm | ✅ passa |
| **60 cm** | ❌ declara PAREDE → **contorna sozinho** |

O que na fase anterior era "efeito colateral do fix" aqui vira **feature**: o robô nunca
tenta espremer em 60 cm por conta própria, nunca raspa, e ainda assim cumpre a missão.
**4 dos 5 obstáculos já estão entregues com o código de hoje.**

### Nível 2 — o atalho de 60 cm (`door_crossing` generalizado)

Módulo que, ao ver a fresta apertada no caminho, **assume o controle do nav2**, alinha em
malha fechada e atravessa reto, devolvendo o controle do outro lado.

É a máquina two-phase do `door_crossing.py`, que a memória do projeto marca como
**"FUNCIONA, não reescrever"** (validada em porta real em campo). A generalização é de
"porta do mapa" para "qualquer fresta detectada", com o lateral apertado de 8 → 3 cm.

### Aproximação final ao cone — NÃO é trabalho do nav2

⚠️ **Conflito previsto:** o cone é obstáculo no costmap. Cone (raio ~0,17) + `robot_radius`
0,32 = fronteira letal a **0,49 m** do centro do cone, com inflação global de 0,60 por cima.
Um goal a 0,60 m do cone cai **dentro da inflação** → o nav2 recusa
(`"Either of the start or goal pose are an obstacle!"`, que já é bug aberto conhecido).

**Desenho:** nav2 leva até **~1,0 m** do cone (fora da inflação). Dali, uma **aproximação
final** guiada pelo `cone_detector` dirige reto até a distância-alvo e para. Mesmo padrão
do atalho de fresta: nav2 para o grosso, comportamento dedicado para o fino.

`dist_alvo_cone` = **0,60 m** do centro do robô ao centro do cone
(0,17 do cone + 0,20 de folga + 0,25 da frente do robô ≈ 0,62). Parâmetro, não constante.

### LED / relé

Nó publica o acionamento do relé quando um waypoint é marcado. Firmware na MEGA
(pino a definir — ver `project_mega_pinout`). Hardware ainda vai ser remontado pelo dono.

---

## 5. O mundo do sim — `worlds/arena_galpao.sdf`

Galpão de **14 × 9 m** com paredes (o dono confirmou galpão fechado → AMCL tem estrutura).
Espelha o estilo de `sala_grande.sdf`: caixas SDF escritas à mão, `<static>true</static>`.

### Pontos

| ponto | posição (x, y) |
|---|---|
| Largada | 1,0 · 1,0 (rumo +x) |
| Cone 1 | 4,5 · 1,5 |
| Cone 2 | 11,5 · 1,8 |
| Cone 3 | 12,2 · 7,5 |
| Cone 4 | 5,0 · 7,8 |
| Chegada | 1,5 · 2,5 |

Cone = cilindro r 0,17 × h 0,70 (passa no filtro 4-45 cm do `cone_detector`).
Plataforma = placa amarela 1,2 × 1,2 × 0,01, **sem colisão** (é marca de chão, invisível
ao laser — de propósito).

### Obstáculos (cada um com contorno viável)

| obst | perna | eixo | fresta | contorno |
|---|---|---|---|---|
| A | cone 1 → 2 | x = 7,5 | **90 cm** (y 1,80–2,70) | por y > 4,20 |
| B | cone 2 → 3 | y = 4,6 | **70 cm** (x 11,05–11,75) | por x < 9,55 |
| C | cone 3 → 4 | x = 8,2 | **60 cm** (y 7,20–7,80) | por y < 5,40 |
| D | cone 4 → chegada | y = 4,6 | **80 cm** (x 2,20–3,00) | por x > 4,60 |

Blocos com 0,80 m de altura — bem acima do laser a 0,465, visíveis em qualquer pose.
Cada par de blocos é uma **ilha**: a ponta encostada no muro deixa < 0,30 m (vedada),
a outra ponta fica aberta e é o desvio.

Mapa `.pgm`/`.yaml` gerado do mundo, conferido com `tools/mapa_passagens.py`
— que **deve** acusar a fresta de 60 como fechada para raio 0,32. Isso é o esperado, não bug.

---

## 6. Fora de escopo até 05/09 (decisão do dono)

> *"acho que não conseguimos nem a rampa e nem o obstáculo movél a tempo... esse trampo é pra meses"*

### 🚫 Rampa 60×60 @ 15° (sobe → plataforma → desce)

Levantado nesta sessão, registrado para não se perder:

- Sobe **16,1 cm** (60·tan 15°). O LiDAR está a **46,5 cm** → **a rampa é invisível no plano**.
  O robô entraria nela cego.
- **Inclinado 15°, o plano do laser aponta para o chão e bate a 1,74 m** (0,465/tan 15°)
  → parede fantasma no meio da subida → o nav2 aborta.
- Descendo, o laser aponta para cima e não vê nada — cego para a frente.

Tratamento futuro: filtrar o scan pelo pitch da IMU + marcar a região como transponível.
**Não é ajuste de parâmetro.**

### 🚫 Barreira móvel oscilante

Barreira de ~60 cm deslizando numa porta de ~1,2 m; no extremo, sobra janela de 60 cm.
Exige rastrear a barreira, prever a janela, **esperar** e commitar a travessia.
Depende do atalho de fresta estar pronto (a janela é a mesma fresta de 60, porém móvel).

---

## 7. Ordem de trabalho

| # | passo | entrega |
|---|---|---|
| 1 | Fix `track_width` 0,50 → 0,44 na URDF real | premissa de tudo |
| 2 | `worlds/arena_galpao.sdf` + mapa + conferência | o campo de prova |
| 3 | **Baseline no sim**: missão só com nav2 puro, contornando o 60 | garante A1–A5 |
| 4 | Aproximação final ao cone (`dist_alvo_cone`) | garante A2 |
| 5 | LED / relé (nó + firmware) | garante A3 |
| 6 | Atalho de 60 cm (`door_crossing` generalizado) | o bônus |
| 7 | Deploy na Pi + SLAM do galpão real | a prova |

Passos 1–5 **entregam a missão**. 6 é o que ganha tempo. 7 é o dia.

---

## 8. Riscos

| risco | mitigação |
|---|---|
| Erro do AMCL > 3 cm mata a fresta de 60 | Medir no real antes de confiar. A missão não depende disso (A5). |
| Bug aberto `"goal/start is an obstacle"` gera recovery | Já existia antes desta fase. Aproximação final fora do nav2 (§4) tira o goal de dentro da inflação — pode até reduzi-lo. |
| Plataforma amarela invisível ao laser | Por desenho: o cone é a âncora, não a plataforma. |
| Hardware do LED ainda não montado | Nó e firmware ficam prontos e testáveis sem ele; integra quando o relé subir. |
| SLAM do galpão sai ruim | Conferir com `mapa_passagens.py` **no dia**, antes de rodar. |

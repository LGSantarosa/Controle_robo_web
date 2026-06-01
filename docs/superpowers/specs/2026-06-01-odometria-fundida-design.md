# Odometria fundida unificada (rodas + IMU + flow) com degradação graciosa

Data: 2026-06-01
Status: aprovado (design)

## Problema

No modo SLAM/Nav2, o `slam_toolbox` e o AMCL consomem o TF `odom→base_link` e o
tópico `/odom` publicados pelo `odom_publisher`, que é **odometria de roda pura**:

```python
angular = (v_right - v_left) / self.wheel_base   # odom_publisher.py:120
```

Num robô **skid-steer de 4 rodas** o yaw derivado da diferença de rodas é a pior
fonte possível de rotação: ao girar, as rodas arrastam/derrapam lateralmente, então
a rotação real ≠ cinemática. Soma com os defeitos conhecidos de fiação do feedback
traseiro (swap L/R, sinais — ver memória `project_wheel_feedback_wiring`). Resultado
observado em campo: ao virar, o palpite de yaw que chega no `slam_toolbox` está errado,
ele não casa o scan novo com o anterior e carimba "paredes fantasma" giradas — "ele
virou e acha que o mundo girou".

Já existe um `pose_estimator` que **funde os 3 sensores** (IMU/BNO055 + flow/PMW3901 +
rodas), mas hoje ele:
- **exige a IMU**: sem `/imu/data` faz `return` e não publica nada (`pose_estimator.py:261`);
- **não publica TF** (por design, docstring linha 15);
- **só roda no modo trekking**.

Ou seja, a fusão boa existe mas não chega no SLAM, e do jeito atual ela morre sem IMU.

## Realidade técnica que delimita o escopo

O **flow (PMW3901) não mede rotação** — um sensor óptico apontado pro chão mede
translação (dx, dy), não giro. As únicas fontes de yaw são **IMU** (boa, imune a
derrapagem) ou **diferença de rodas** (ruim no skid-steer).

Consequência para o **odom** (TF `odom→base_link`): sem IMU, o flow melhora a deriva de
**translação**, mas a **curva** continua dependendo do yaw de roda. A IMU é o conserto
real do yaw do odom. Sem ela, o único lever sobre o yaw do odom é calibrar o
`wheel_base` efetivo.

### Mas o LiDAR também dá yaw — dentro do slam_toolbox

Há uma segunda via, independente do odom: o **scan matching** do slam_toolbox casa o
scan novo com o mapa/scan anterior e daí extrai movimento, **inclusive rotação**.
Observado em bancada: deslocamentos pequenos do LiDAR são acompanhados quase
perfeitamente no mapa. O odom é apenas o *palpite inicial* que semeia esse casamento.
A parede fantasma surge quando o palpite de yaw de roda vem muito errado na curva
(semente ruim) ou quando o gate `minimum_travel_heading` (default ≈ 0.5 rad ≈ 28°)
pula scans durante o giro.

Caveat: yaw por LiDAR é **dependente do ambiente** — ótimo indoor (paredes = features),
fraco em campo aberto (sem referência), justamente onde o trekking opera. A IMU não
tem essa fraqueza. Por isso aproveitamos o LiDAR para o **mapeamento** (afinar o
slam_toolbox), mas ele não substitui a IMU no odom para Nav/trekking ao ar livre.

## Objetivo

Um nó único de odometria que publica `/odom` (nav_msgs/Odometry) + TF `odom→base_link`,
fundindo rodas + IMU + flow com **degradação graciosa** (usa só os sensores presentes),
consumido por SLAM, AMCL e Nav2 em todos os modos reais.

## Não-objetivos (YAGNI)

- Não trocar a fusão de translação flow⊕roda existente (já validada).
- Não usar `robot_localization`/EKF (a fusão custom já existe e o time a entende).
- Não mexer no `sim.launch.py` (no Gazebo o plugin DiffDrive já dá odom/TF).
- Não separar agora a camada trekking (cone pose_fix / `/trekking/*`) do núcleo de
  odometria — fica como cleanup futuro anotado, não neste escopo.
- **Não adicionar o nó de odometria por LiDAR (rf2o/laser_scan_matcher) nesta entrega**
  — fica como **fase 2** (ver seção própria). Gatilho: se, após a afinação do
  slam_toolbox e a fusão IMU/flow, o **Nav2/AMCL** ainda precisar de yaw confiável sem
  IMU. Custo evitado por ora: dependência nova na Pi, CPU, e fragilidade em campo aberto.

## Decisões resolvidas

- **(A) Aposentar o `odom_publisher` do launch.** "Só rodas" vira o caso degenerado do
  nó fundido (IMU e flow ausentes → comportamento idêntico ao `odom_publisher` atual).
  Um nó, um `/odom`, um TF — elimina o conflito de dois publicadores do mesmo TF.
  O arquivo `odom_publisher.py` permanece no repo como referência, fora do launch.
- **(B) Na queda de IMU em movimento: snap duro** pro yaw de roda (a partir do último
  yaw conhecido), com log. Sem blend suave.

## Arquitetura

Evoluir o `pose_estimator` no **nó único de odometria**:

- Publica `/odom` (nav_msgs/Odometry) + TF `odom→base_link`.
- Mantém `/trekking/pose`, `/trekking/odom`, `/trekking/slip`, `/trekking/health`
  para compatibilidade com `trekking_runner`/`cone_detector` (inalterados).
- Lançado pelo `robot.launch.py` no lugar do `odom_publisher` (drop-in, mesmo ponto),
  então SLAM e Nav2 herdam a fusão automaticamente.

### Fusão com degradação graciosa

- **Yaw** — watchdog em `/imu/data` (parâmetro `imu_timeout`):
  - IMU fresca → yaw absoluto do quaternion da IMU; `yaw_rate` do gyro.
  - IMU ausente/velha → integra `yaw += angular_wheel · dt`, com
    `angular_wheel = (v_right − v_left) / wheel_base` usando o **wheel_base efetivo
    calibrado**. `yaw_rate` = `angular_wheel`.
  - Remove o gate `if not self.have_yaw: return`.
- **Translação** — mantém a fusão flow⊕roda existente (peso α = sigmoid sobre quality,
  watchdog `flow_timeout`). Flow ausente/ruim → roda sozinha.

### Cenários

| IMU | Flow | Yaw | Translação |
|-----|------|-----|------------|
| ✓ | ✓ | IMU | flow⊕roda |
| ✗ | ✓ | roda (calibrado) | flow⊕roda |
| ✓ | ✗ | IMU | roda |
| ✗ | ✗ | roda (calibrado) | roda (= `odom_publisher` atual) |

### Transições

Troca dura com log único na borda: IMU cai → integra yaw de roda a partir do último
yaw conhecido; IMU volta → yaw snap pro absoluto (degrau pequeno, logado). No teste de
hoje (sem IMU desde o boot) não há transição — usa o fallback de roda o tempo todo.

## Mudanças de launch

- `robot.launch.py`: lança o nó fundido no lugar do `odom_publisher`; repassa
  `wheel_radius`, `wheel_base`, `left/right_wheel_sign`, e os novos `imu_timeout`,
  `flow_timeout`, params de flow.
- `trekking.launch.py`: não sobe mais o `pose_estimator` à parte (já está na base).
- `sim.launch.py`: sem mudança.

## Afinação do slam_toolbox (parte do escopo — conserta o mapa)

Ataca diretamente a parede fantasma no mapeamento, fazendo o slam_toolbox confiar no
próprio scan matching em vez de na semente de yaw de roda. Em `slam.launch.py`:

- **Baixar os gates de travel**: `minimum_travel_heading` 0.5 → ~0.1–0.15 rad e
  `minimum_travel_distance` 0.5 → ~0.1–0.2 m. Processa scan em incrementos pequenos
  na curva → o erro de yaw por passo fica pequeno → o matcher (Ceres) converge bem
  mesmo com semente de roda ruim. É o lever dominante.
- Manter `use_scan_matching: true`; se necessário, aumentar `scan_buffer_size` e
  revisar os params de busca correlativa.

Valores são pontos de partida, refinados no teste de mapeamento (hands-on). Custo: um
pouco mais de CPU na Pi (mais scans processados) — aceitável.

## Fase 2 (fora deste escopo): nó de odometria por LiDAR

Se após a afinação acima + fusão IMU/flow o Nav2/AMCL ainda precisar de yaw confiável
sem IMU: adicionar `rf2o_laser_odometry` (ou `scan_tools`/`laser_scan_matcher`) como
fonte de yaw na fusão, prioridade **IMU > LiDAR-odom > roda**, com watchdog/gate de
qualidade pra cair pra roda quando os features somem (campo aberto). Requer buildar a
dependência na Pi e medir o custo de CPU. Não implementar agora.

## Calibração do wheel_base efetivo (hands-on, etapa separada pós-código)

Procedimento, com o robô se movendo (anunciar e aguardar "pode" antes):
1. Garantir que está no fallback de roda (sem IMU) e zerar a pose.
2. Comandar rotação pura por uma volta completa marcada fisicamente (360° reais).
3. Comparar o yaw integrado de roda com 360°.
4. Ajustar `wheel_base_eff = wheel_base_atual · (yaw_integrado / yaw_real)` e repetir
   até bater (skid-steer: a bitola efetiva fica maior que a geométrica de 0.50 m).
5. Gravar o valor calibrado como default no `robot.launch.py`.

## Testes (TDD)

Testes unitários do nó fundido:
- sem IMU → yaw vem do fallback de roda (publica, não trava);
- com IMU → yaw vem da IMU;
- flow stale → α→0 (só roda na translação);
- nada presente → saída idêntica à roda-pura do `odom_publisher`;
- queda de IMU em movimento → snap pro yaw de roda sem descontinuidade de tempo.

Rodar `colcon test` + os 9 testes já existentes antes de fechar.

## Riscos

- **Nó fundido com bug derruba odom em todos os modos** (antes o `odom_publisher` era
  simples). Mitigação: o caso degenerado é exatamente a lógica antiga; testes cobrem.
- **Mistura da camada trekking no nó geral** (cone pose_fix sempre carregado). Inócuo
  fora do trekking (`/trekking/pose_fix` nunca publicado). Anotado como cleanup futuro.
- **Distinguir mapa de odom sem IMU**: o **mapa** melhora na curva hoje (afinação do
  slam_toolbox + scan matching, indoor). Já o **yaw do odom** (Nav2/AMCL, e qualquer
  ambiente pobre em features) só melhora com a calibração do `wheel_base` até a IMU
  voltar — a IMU é o conserto definitivo e geral do giro. Expectativa alinhada com o
  usuário.

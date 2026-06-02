# SLAM recupera prior de yaw ruim no giro (robô sem IMU)

Data: 2026-06-02
Status: aprovado (design)

## Problema

Confirmado em campo (2026-06-02): **o robô não tem IMU**. Logo o `pose_estimator`
roda permanentemente no fallback de roda (`imu_fresh == False` em
`pose_estimator.py:291`), e o TF `odom→base_link` carrega yaw derivado só da
diferença de rodas:

```python
angular = (v_right - v_left) / wheel_base   # fused_odom.py:30
```

Num skid-steer de 4 rodas isso é a pior fonte de rotação possível: ao girar as
rodas derrapam, então o yaw cinemático ≠ real (some com swap/sinais do feedback
traseiro — ver memória `project_wheel_feedback_wiring`).

Sintoma observado: **mapa sai ótimo no reto e a pose se perde completamente no
primeiro giro.** Diagnóstico:

- A web mostra a pose **corrigida** `map→base_link` (`map_service.py:197`,
  `lookup_transform('map','base_link')`), não a odom crua. Como o mapa saiu limpo
  até o giro, o slam_toolbox *estava* rastreando — e a correção dele divergiu no
  giro. Não é "só mostrando odom".
- O slam_toolbox semeia a busca do scan matcher na pose prevista pelo odom. Com a
  semente de yaw de roda torta no giro, o match real cai **fora da janela de busca
  angular** (default `coarse_search_angle_offset` ≈ ±20°) → ele casa errado ou não
  casa → `map→odom` diverge → a pose voa.
- O c13e739 já baixou os travel gates (processa scans em passos menores), o que
  ajudou no reto mas **não alargou a busca angular nem ligou a expansão de
  resposta** — então o giro continua estourando a janela.

## Realidade técnica que delimita o escopo

- **Sem IMU, não há fonte boa de yaw no odom.** Flow (PMW3901) mede translação, não
  giro. A diferença de rodas é a única fonte de yaw do odom, e é ruim no giro. Não
  vamos consertar o odom — vamos deixar o **LiDAR (via scan matching do
  slam_toolbox)** ser o dono da pose corrigida.
- **Não mexer no `pose_estimator`.** Tentamos meter o LiDAR no `pose_estimator`
  (2026-06-01) e não foi bom. O `pose_estimator` fica como está (roda + flow); ele
  só fornece o *palpite inicial* (seed) — o slam corrige por cima.
- **Limites inerentes (não dá pra tunar pra fora):**
  1. Rotação por LiDAR é ambígua em corredor longo simétrico / campo aberto sem
     feature. Janela larga não resolve ambiguidade de ambiente. (Por isso isto é
     fix de **mapeamento indoor**, não substitui IMU pra Nav/trekking ao ar livre.)
  2. Busca larga + expansão custam CPU na Pi no instante do giro; de olho no
     `transform_timeout: 0.5` (se o match atrasar, o TF lagueia).

## Objetivo

No modo mapping, o slam_toolbox recupera o casamento de scan mesmo quando a semente
de yaw de roda vem errada no giro — alargando a janela de busca angular, ligando a
expansão de resposta quando o match vem fraco, e processando mais scans durante a
rotação. Resultado: `map→base_link` cola na realidade no giro e o mapa não duplica
parede.

## Solução

Mudança **concentrada no bloco de parâmetros do `slam_toolbox` em
`ros2_packages/robot_nav/launch/slam.launch.py`**. Nenhum nó novo, nenhum arquivo
de código Python alterado.

### Primárias (o coração do fix)

```python
'use_response_expansion': True,        # match fraco (seed de yaw errado no giro) →
                                       # expande a janela de busca progressivamente
                                       # até achar; custo de CPU só quando precisa.
'coarse_search_angle_offset': 0.6,     # ±~34° (era ±~20° default) — cobre seed de
                                       # yaw torto no spin.
```

### Secundárias (folga + processar mais no giro)

```python
'minimum_angle_penalty': 0.7,              # era 0.9 (default) — penaliza menos
                                           # correção angular grande contra o seed ruim.
'correlation_search_space_dimension': 0.6, # era 0.5 (default) — folga de busca linear.
'minimum_travel_heading': 0.10,            # era 0.12 — mais scans durante o giro
                                           # (passo menor → erro de seed menor por passo).
'minimum_time_interval': 0.1,              # era 0.2 — até ~10 Hz de processamento no spin
                                           # (LD06 publica a ~10 Hz).
```

Mantém o que já está: `use_scan_matching: True`, `minimum_travel_distance: 0.15`,
`scan_buffer_size: 20`, frames e tópicos.

### Fluxo

```
/scan (LD06) ─┐
              ├─► slam_toolbox scan matcher (janela angular larga + response expansion)
TF odom→base_link (seed de yaw de roda, torto no giro) ─┘
              │
              ▼
        map→odom (corrige o erro de yaw do giro)
              │
   map→base_link = (map→odom) ∘ (odom→base_link)  ──► pose que a web mostra
```

## Verificação

Tuning empírico — **só fecha na bancada** (robô está offline no momento deste
design). Sem teste automatizado possível (depende de hardware + ambiente).

Plano de validação (hands-on — anunciar e esperar "pode" do usuário antes de abrir
a captura, ver memória `feedback_announce_before_test`):

1. Subir o stack de SLAM (rig de bancada em `/tmp/bench_slam.sh`, ver memória
   `project_lidar_slam_bench`), UI web em `ROBOT_MODE=slam`.
2. Dirigir reto → girar (incluindo spin no lugar) → reto.
3. Observar:
   - a pose `map→base_link` na web **não voa** no giro;
   - o mapa **não duplica parede** ("parede fantasma") depois do giro;
   - CPU da Pi e logs do slam_toolbox no giro (sem warns de TF lento /
     `transform_timeout` estourado).
4. Critério de sucesso: completar um circuito com pelo menos um giro fechado e o
   mapa permanecer coerente, com a pose colada.

Se ainda escorregar: subir `coarse_search_angle_offset` mais, ou baixar
`minimum_time_interval`, vigiando CPU.

## Fora de escopo

- Mexer no `pose_estimator` / `fused_odom` (decisão explícita do usuário).
- Adicionar nó de laser-odometry (rf2o / laser_scan_matcher).
- Calibrar `wheel_base` efetivo (lever de odom; pode entrar depois como complemento
  barato, mas não neste spec).
- Qualquer coisa de Nav2/AMCL/trekking ao ar livre — isto é fix de mapeamento indoor.

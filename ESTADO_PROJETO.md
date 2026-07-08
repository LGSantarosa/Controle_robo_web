# Estado do Projeto — Controle_robo_web

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Acessível de qualquer PC (está versionado na `main`). Atualizado em **2026-07-07**.

---

## 🆕 2026-07-08 — Câmera POV (Logitech C922) ✅ VALIDADA EM CAMPO no mesmo dia

> **Deployada na Pi (`dd6fe9c`) e aprovada pelo dono** ("ficou do caralho, adorei"):
> run real de 2 min gravou sozinha no ▶ da rota, parou sozinha, vídeo perfeito.
> 294 MB / 2 min ≈ 2,5 MB/s ≈ **4,4 GB por run de 30 min** (SD da Pi: 12 GB
> livres — atenção em runs longas). Setup feito na Pi: ffmpeg instalado
> (precisou `dpkg --configure -a` antes) + `robo` no grupo `video`.
> Melhoria futura opcional: gravar 960x540 ou H.264 pelo encoder de HW da Pi
> (bcm2835) → ~0,5 GB/run. `pov_*.mkv/.mjpeg` no .gitignore.

- `controle_web/camera_service.py`: ffmpeg dono do /dev/video0, MJPEG **copy**
  (zero re-encode, poupa a CPU da Pi), 720p@30. Thread separa os frames e
  serve o live view + grava `logs/pov/pov_<data>_<gatilho>.mjpeg`; ao parar,
  remux em background pra `.mkv` (cópia) e apaga o cru.
- **Gravação segue a run**: inicia no ▶ Iniciar (rota) e no 🎯 Ir para; cada
  goal aceito pelo Nav2 mantém viva (hook `on_nav_start/end` no nav_metrics);
  fim de goal arma debounce de 8 s (não picota entre waypoints); ■ Parar corta
  na hora; teto de 30 min (cobre goal via porta, que usa navigate_through_poses
  e o nav_metrics não rastreia).
- **GUI**: botão `🎥 POV` nas Camadas (aparece se o serviço subiu; desabilitado
  sem câmera) abre PiP sobre o mapa com `● REC` piscando; stream `/camera/stream`
  limitado a ~10 fps pro WiFi (gravação segue 30 fps cheios).
- Sem câmera/ffmpeg: serviço fica quieto, replug reativa sozinho (retry 5 s).
- 15 testes novos em `test_camera_service.py` (parser MJPEG + máquina de
  estados da gravação); suíte do controle_web passando.
- Pra usar: subir `robot-up nav2` normal — card **🎥 Câmera (POV)** na GUI
  (Ligar = live view), gravação automática na run, vídeos em
  `controle_web/logs/pov/` na Pi (puxar via ssh/scp e **apagar da Pi** depois,
  senão o SD enche em ~3 runs longas).

---

## 2026-07-07 (tarde) — GUI redesenhada + mapa hotmilk + rede de campo

> Tudo DEPLOYADO na Pi e aprovado pelo dono no mesmo dia. Commits `b37e11e..68daca7`.

- **GUI console**: topbar sticky (chips socket/energia/controle/pose + E-STOP),
  layout 2 colunas (mapa protagonista; 1 coluna em teleop), cores por zona
  (controles índigo, velocidade âmbar, mapa ciano), toolbar do mapa agrupada
  (Rota/Arquivos/Camadas c/ toggles ON/OFF), HUD de v/ω no canvas (derivado da
  pose no cliente), trilha laranja #f90 desbotando.
- **Zoom/pan no mapa** `284b3b5`: roda = zoom no cursor, botões +/−/⤢, pinça no touch.
- **Arrastar = mover o mapa** `68daca7`: goal único virou botão **🎯 Ir para**
  (one-shot, desarma após enviar) — fim do goal acidental ao mexer no mapa.
- **Mapa novo `hotmilk`** (598KB, maior que o sala): no repo `d7b4d21`; backup do
  golden ativo `sala_boa_2026-07-07.*` chmod a-w na Pi. **rota1 validada no
  hotmilk** (mesma origem do SLAM). Subir: `robot-up nav2 --map=maps/hotmilk.yaml`
  (default do launch.sh AINDA é sala.yaml).
- **Rede**: hotspot "Trafico de banana" autoconnect prioridade 100 (no campo:
  ligar o hotspot ANTES do robô; acessar via robo-desktop.local no hotspot) +
  IP fixo 192.168.18.95 na rede de casa.
- BO conhecido (cosmético): traceback do waypoint_runner no shutdown
  (`map_service.py:1125` — wait_for_service com nó já morto durante retry);
  fix de 3 linhas pendente, só aparece derrubando o app com waypoints ativos.

---

## 2026-07-07 — 8ª auditoria FEITA + 5 itens APLICADOS (A1..A5)

> `AUDITORIA_2026-07-07.md` na raiz. Nenhum bug crítico — achados de resiliência.
> **Aplicados no mesmo dia (autorizado): A2 `42981e4` (nearest stale no unstuck),
> A1 `e69c50f` (motion_guard respawn=True), A3 `3b8967b` (default do launch =
> _pi.yaml; fóssil → _legacy), A4 `3b1ba9b` (guard não snapshotta girando, +1
> teste), A5 `2d70bf6` (flush CSV em timer 2s).** 246 testes, build OK, smoke
> test 5s nos 3 nós OK. ⏳ A4 validar no sim com a "pessoa" teleop antes da Pi;
> deploy na Pi = `git fetch && reset --hard origin/main` + colcon robot_nav.
> A6 aceito como está; A7 só se a CPU pedir; B1 (untracked) pendente do dono.

---

## 🏆 2026-07-06 — AS 2 MELHORES RUNS DA HISTÓRIA: vilão das pausas FECHADO em campo

**Deploy `8a76116` na Pi** (limit `581f02c` + persistence `ac3cd24` + time_step
0.02 `8ba8b7a`, build OK) → rota padrão em **LOOP, ~5 ciclos completos, 26min de
goal ativo, ZERO erro** — com o dono + outra pessoa atrapalhando de propósito
(entrando na frente, passando perto, forçando desvio). "Caralho ele deu um baile."

### Números (pause_budget do freeze_capture, janela de 31min)
- **wz_engolido: 27,9s (1,8% do goal) — era 107s/315s (34%) em 07-03. VILÃO MORTO.**
- Parado com goal: 279s (18%), mas 116s é idle inicial pré-missão → pausa real ~10,5%.
- guard_hold 50s em 53 intervenções = motion_guard freando pra GENTE (comportamento
  desejado, o teste era esse). unstuck 6,4s; collision 5,6s; pior episódio 12s
  (vs 445s do bolsão antigo).
- Inversões de giro ~7/min na janela toda — inclui desvios de gente; sem alarme.

### 🏆🏆 RUN 2 (mesmo dia, ~17h) — "melhor ainda, rápido e decisivo"
Mesma stack, dono atrapalhando de novo (mais agressivo: guard_hold 73s em 634s
de goal). A navegação em si quase não pausou: **wz_engolido 6,0s (1,0%),
vx_zona_morta 2,7s, unstuck 0,1s, collision 1,5s**. Episódio de **31s em t+626
com cmd de avanço e odom=0**: ✅ RESOLVIDO 07-07 — o dono tinha DESLIGADO os
motores no fim da run. Não é BO, descartado. CSVs arquivados na Pi:
`controle_web/logs/run_2026-07-06_best/` (run 1) e `run_2026-07-06_best2/` (run 2).

### Régua daqui pra frente
**Esta run é o BASELINE.** Mudança que piorar pausa real >10,5% ou wz_engolido >2%
em loop equivalente = regressão. Bônus deployado junto: `simulation_time_step
0.1→0.02` no approach (banda cega de rotação por aliasing, achada no A/B do sim
07-06 — margem −11mm virou +7mm).

---

## 2026-07-03 (tarde) — CAÇADA AO VILÃO DAS PAUSAS: causa achada, fix escolhido (✅ validado 07-06)

> **⚠️ PRIMEIRA AÇÃO SEGUNDA (2026-07-06): a Pi está ATRASADA.** main = `581f02c`
> (limit REATIVADO) mas a Pi ficou em `75ff844` (sem limit) — o robô desligou antes
> do deploy. **Antes de ligar a stack: `git fetch && git reset --hard origin/main`
> + `colcon build --packages-select robot_nav` na Pi.**

### O VILÃO das pausas (medido com pause_budget + freeze_capture, decisão do dono: LIMIT fica)
- **Causa raiz PROVADA**: o `PolygonSlow` do collision_monitor (action `slowdown`,
  ratio 0.3) escala TAMBÉM o giro-no-lugar: follower manda wz=2.4 → collision entrega
  0.72 → **abaixo da zona-morta 1.7 = giro morto**. 34% do tempo de comando da missão
  (107s/315s) era giro comandado que não acontecia. **Fecha o BO pausado dos "giros
  >5s"** (a resposta de "onde 2.4 vira 0" era o PolygonSlow).
- **Anatomia do ponto-problema (bolsão ~(5.1,-1.16) do mapa `sala`, 445-452s preso 2×)**:
  o robô entra no bolsão por um corredor estreito e precisa de um **giro de ~180°**
  pra sair por outra passagem estreita (observação do dono: "se ele completa o 180
  ele sai de boa"). Girando no bolsão, o nariz VARRE as paredes (0.3-0.5m) → quando
  cruza o setor da parede o PolygonSlow mata o giro NO MEIO → fica apontado pra
  parede, alvo ~180° atrás, ruído troca o lado do giro (54 inversões medidas),
  unstuck e follower brigam. Com o `limit` ativo esse ponto custou **10s** (vs 445s).
- **Fix = PolygonSlow action `slowdown` → `limit`** (corta POR EIXO): `linear_limit
  0.10` (mesma proteção de aproximação, 0.35×0.3) + `angular_limit 4.0` (giro NUNCA
  capado; atuador satura 2.5; PolygonStop/approach continua zerando colisão real).
  Commit `581f02c` (reapply de `dbe0c78`). **⏳ VALIDAR no ponto-problema segunda.**

### A saga do dia (pra não repetir os erros)
1. `dbe0c78` limit aplicado → **run ficou em ZIGUE-ZAGUE** (inversões de giro 4.1→8.1/min,
   girando 11%→23%) → revertido (`3571a3e`). Diagnóstico do zigue-zague: com o giro
   destravado o follower executava cada balançada do replan de 1Hz.
2. **Por que o plano balançava ("plan enlouquecendo", dono)**: dropout de feixe rasante
   do LD06 fazia a marca de obstáculo PISCAR no costmap → Theta* trocava de LADO a cada
   replan (31 flips de lado em 781s SEM nada na frente). **Fix `ac3cd24`:
   `observation_persistence 0→1.0s` (local+global)** — flips caíram pra 8. O motivo do
   0.0 de 06-08 (fantasma <0.15m preso 1s) hoje morre no scan_safe. ✅ DEPLOYADO+testado.
3. `6d34714` compromisso de rumo no follower (commit_dist 0.35m + turn_enter_committed
   35° + sticky_behind 150°) → **em campo NÃO mudou nada no ponto-problema** (depois do
   unstuck girar o robô, o erro é sempre >35° → turning re-entra "legítimo" com alvo
   fresco do plano) → revertido a pedido (`75ff844`). ⚠️ O código está na história do
   git — o **sticky_behind (lado grudento p/ alvo ~180°)** ainda é candidato válido se
   sobrar alternância de lado DEPOIS do limit validado.
4. **Lição de método**: limit sozinho = zigue-zague; persistence sozinho = não destrava
   o 180 do bolsão. **A dupla limit+persistence é o pacote** — testamos o limit no
   ambiente errado (antes do persistence) e quase jogamos fora o fix certo.

### Estado dos knobs/ferramentas novas de hoje (tarde)
- `bin/pause_budget.py` (offline): orçamento do tempo parado por causa + episódios.
  Usa o freeze_capture.csv turbinado (`843388e`: follow_vel, auto_vel_pre, unstuck_vel,
  follow_state, guard_state, goal_active na col. extra).
- Régua pro teste de segunda (rota padrão + foco no bolsão):
  - bolsão: tempo preso (era 445s; com limit era 10s)
  - zigue-zague: inversões de giro/min no odom (bom=4.1, ruim=8.1)
  - flips de lado do plano (bom=8/781s)
  - pause_budget: wz_engolido (era 107s) e "outro" (era 96s)

---

## 🆕 2026-07-03 — motion_guard ✅ APROVADO no real ("isso ta bom pra caralho") + anti-livelock do giro do unstuck

> Tudo na `main` e **deployado+buildado na Pi** (`b589b42`). Sessão de campo com o dono.

### motion_guard: falsos positivos RESOLVIDOS (3 fixes, 1 rodada limpa por fix)
Dono reclamou "lento demais perto de parede/objeto MAPEADO". CSV provou: **100% do tempo em
slowing/blocked SEM ninguém perto** (flicker re-armava o clear_time a cada ~0,85s). 3 causas
distintas, cada uma provada por CSV antes do fix:
1. **`persist_frames=3`** (`277b398`): só latcha com 3 scans consecutivos vendo móvel — flicker
   de 1 frame (TF atrasado/oclusão) era 62% dos falsos. Custo ~0,3s de latência. 100%→71% ativo.
2. **Raycast "estava LIVRE mesmo?"** (`ae8ccff`): célula ausente no snapshot ≠ livre (pode estar
   na SOMBRA de um objeto). Snapshot agora guarda pose + mapa polar (bin 1°→alcance); móvel só se
   o feixe velho ATRAVESSOU a célula. Mata a borda de oclusão. 71%→46%.
3. **Polar do scan COMPLETO** (`b589b42`): CSV diagnóstico (colunas novas px,py,pyaw,vx/wz_odom,
   cx,cy,cbear_deg em `38de948`) mostrou 25% das detecções com robô PARADO, atrás/do lado, nos
   MESMOS lugares = **dropout de feixe rasante do LD06** (some segundos e volta; ao voltar
   parecia móvel). Feixe inválido agora entra como alcance 0.0 = DESCONHECIDO (nunca "livre").
**Resultado final (CSV + dono): rota solo SEM 1 freio falso (48s, 0 episódios); com gente
passando, 24 episódios de 4-9s soltando sozinho + 2 longos (20/30s, pessoa insistindo =
correto). Dono: "isso ta bom pra caralho".** 245 testes. Knob novo live: `persist_frames`,
`ray_bin_deg`. ⚠️ Colunas de diagnóstico do CSV ficam até o guard assentar (remover depois).

### unstuck: giro calculado (clear_turn) loopava — anti-livelock `turn_escape_after=2` (`9fcc653`)
Campo: robô girou ~10× no lugar sem sair (log: 9× `turning reason=mapped GIRO_CALC=+6..12°` no
mesmo ponto, ~45s). O clear_turn vinha ANTES da ré e **não entrava em nenhum histórico**
anti-loop. Agora tem `turn_history` (mesmo padrão spin/move): 2 tentativas no mesmo ponto
(0,5m/120s), na 3ª pula o giro e cai na ré/avanço/escape. 241→243 testes (na época).

---

## 🆕 2026-07-02 (tarde) — Fluidez P1+P2 no path_follower + motion_guard NOVO (validado no sim)

> Tudo na `main`. ⏳ deploy na Pi = decisão do dono (nada disso foi pro real ainda).

### 1. Fluidez dos giros (spec/plano 07-02): P1+P2 aplicados, medidos na rota do dono
- **P1 `0b35ef2`**: `rot_min` 2.0→2.4 rad/s (2.0 comandado ≈ 10°/s reais na zona-morta 1.7 =
  rastejo que parecia parada; 2.4 ≈ 25°/s). **P2 `7d1fe49`**: alvo do giro CONGELADO ao entrar
  em turning (não caça mais o replan ~1Hz no meio do point-turn). 214→230 testes.
- **Medição (sim sala_grande, rota de 5 pontos do dono, baseline = runs anteriores da mesma
  rota)**: hoje **922s** vs 810s (06-28 com fix do unstuck) e 1665s (06-28 baseline).
  turning 38,7% (real 07-02 dava 56%), driving mediano 0,89s, wz_flips 61.
- **🔴 PRÓXIMO ALVO: giros monstruosos** — 7 giros >5s somando ~135s (piores: 36,8s, 33,2s,
  22,7s). Giro puro a ~24°/s faz 180° em 7,5s → 37s girando = outra coisa segurando
  (suspeitos: collision segurando wz, re-alinhamentos em cadeia). CSV guardado no scratchpad.

### 2b. motion_guard NO REAL 07-02 (tarde/noite) — 🟡 PARCIAL: "deu boa, melhorou", dono quer MAIS testes (seg 07-06?)

4 iterações de campo no mesmo dia (dono andando perto do robô real), tudo deployado na Pi (`16d50fb`):

1. **unstuck standdown** (`e9f41e2` + teto `84b2423`): unstuck disparava `pinch` em ~2s EM CIMA
   do dono (pessoa ≠ parede). Agora: guard em `blocked` → unstuck não conta/dispara; teto
   `guard_hold_max=20s` (bloqueado sem nada mudar → unstuck reativa); soltou → relógio zera.
2. **blocked = parada TOTAL** (`1e68ec3`): wz liberado fazia o robô GIRAR no lugar caçando o
   replan enquanto a pessoa passava. wz agora ZERA no blocked (zerar é seguro; nunca ESCALAR).
3. **bolha `freeze_dist=1.2m`** (`6b00e92`): pessoa do LADO caía em slowing (giro livre, 71% do
   teste) → rodava atrás do plano-contorno. Móvel a <1,2m em QUALQUER direção = parada total.
4. **corredor 2,5m + gap de retomada 3s** (`16d50fb`): dono cruzava o caminho ALÉM do corredor
   de 1,5m → follower saía atrás do desvio-fantasma do planner. Corredor agora cobre o raio
   todo e a retomada espera 3s (~3 replans endireitam o plano antes de andar).

**Veredito do dono: "deu boa, melhorou" — deixar como PARCIAL, testar mais.** Knobs live se
precisar: `guard_radius` (3.5 se cruzar mais longe que 2,5m ainda desviar), `clear_time`,
`freeze_dist`. 237 testes. ⚠️ NUNCA escalar wz parcialmente (zona-morta 1.7).

### 2. motion_guard — cautela com objeto EM MOVIMENTO (pedido do dono pós-run 07-02) ✅ SIM
- **Nó novo** `robot_nav/motion_guard.py` (spec `docs/superpowers/specs/2026-07-02-motion-guard-
  design.md`): diff temporal do `/scan_safe` no frame odom (célula livre 0,5s atrás + retorno
  agora = borda de coisa se movendo; parede/móvel parado não dispara) + clusters. Filtra SÓ a
  autonomia: `twist_mux_auto → auto_vel_pre → motion_guard → auto_vel_raw → collision`.
  unstuck/manual FORA. **wz passa intocado SEMPRE** (escalar giro cai na zona-morta 1.7 =
  congela point-turn). Failsafe: sem TF/scan ou `enabled=false` → pass-through (nunca mata a
  nav). Params live (`ros2 param set /motion_guard ...`). CSV `controle_web/logs/motion_guard.csv`.
- **Comportamento**: móvel no raio 2,5m → vx escala PROPORCIONAL à distância (piso 25% a
  <0,6m — feedback do dono na 1ª validação: 50% uniforme era imperceptível de lado/atrás);
  móvel no corredor à frente (±0,35×1,5m) → PARA e retoma sozinho 1,5s após limpar.
- **Validação no sim (stress de 170s, dono dirigindo "pessoa" de 2 pernas por teleop)**:
  escala mediana 0,25 a <0,8m / 0,44 a 0,8-1,5m / 0,73 a 1,5-2,5m (rampa certinha);
  **11 paradas** de 1,7-4,3s (travessias) e 2 longas (7s/17s = pernas insistindo no corredor,
  correto); nearest mínimo 0,26m SEM bater; retomou sozinho todas as vezes.
- **Ferramentas versionadas**: "pessoa" (2 pernas cilíndricas teleopáveis, VelocityControl)
  no `worlds/sala_grande.sdf` (nasce na sala decoy) + `bin/teleop-pernas` (teclado numérico
  modo carrinho: 8 frente, 4/6 gira, 5 para).
- **Fora de escopo (só se precisar)**: predição de cruzamento por velocidade de cluster
  (proposta B da spec); reação durante point-turn (vx já é 0 e wz é intocável — limitação
  conhecida); atuação no manual.

---

## 🆕 2026-07-02 — As 2 placas "MPU9250" novas TAMBÉM são MPU6500 sem mag (3/3)

> Teste feito num **Arduino avulso (328P) no PC dev** — a MEGA do robô não foi tocada.

- **Veredito (mesmo método do laudo `PROVA_MPU6500_NAO_9250.md`):** as duas placas leram
  `WHO_AM_I(0x75)=0x70` (assinatura do **MPU6500**) e o AK8963 em `0x0C` **não dá ACK** nem
  com bypass → **SEM magnetômetro**. Placar: **3 de 3** módulos "GY-9250" eram 6500 (a
  devolvida + estas duas). O laudo vale igual pra devolução destas.
- **Decisão prática: parar de caçar GY-9250 de marketplace** (mesmo PCB/silk do 6500, chip
  populado quase sempre é o 6500). Pra yaw absoluto, os caminhos confiáveis:
  **magnetômetro dedicado** (QMC5883L/HMC5883L/LIS3MDL, entra no mesmo I²C) ou breakout de
  fornecedor sério (**ICM-20948** Adafruit/Pimoroni, sucessor do 9250). Adaptar o firmware
  do mag (hoje AK8963, dormente) pra QMC5883L é simples.
- **Ferramenta nova VERSIONADA: `firmware/imu_diag/`** (`7db28d8` + env `nano`) — o sketch
  DIAG MAG do laudo, agora no repo: scanner I²C + WHO_AM_I + bypass + WIA do AK8963 +
  amostra em µT, veredito impresso, roda em loop de 3 s (troca de placa sem reflashear;
  cortar o 5V antes). Flash: `pio run -e nano -t upload --upload-port /dev/ttyUSB0`
  (Arduino 328P avulso; SDA=A4, SCL=A5) ou env default pra MEGA (⚠️ apaga o mega_bridge —
  reflashear depois).
- IMU do robô segue o **MPU6050** (fix `553e7b3`, ⏳ validar sinal do yaw na bancada).

---

## 🆕 2026-07-01 — IMU estava MORTA a sessão toda (fix) + PMW3901 arrancado (sem flow temp.)

> Commit `553e7b3` na `main`, **deployado na Pi + MEGA reflashada**. Sessão de diagnóstico.

### 🔴→✅ IMU: o robô rodou a sala INTEIRA (30-06) SEM IMU e ninguém percebeu
- O dono devolveu o "9250" (era MPU6500, sem mag — ver `PROVA_MPU6500_NAO_9250.md`) e
  **recolocou o MPU6050 antigo** (de ponta-cabeça). Mas o firmware direto-por-registrador só
  aceitava WHO_AM_I `0x70/0x71/0x73` (6500/9250/9255); o **6050 responde `0x68`** → `tryInit_`
  rejeitava → `begin()` falhava → `/imu/data` a **0 Hz**. Provado por probe (mesmo nó, mesma
  janela: rodas 48 Hz ✅, IMU 0 Hz, flow 0 Hz).
- **Implicação:** a run "magnífica" de 30-06 e o "erra um pouco na pose às vezes" foram **com a
  IMU MORTA**. Andou bem porque **wheels + LiDAR/AMCL** carregam: o AMCL casa o scan ~10 Hz e
  corrige o yaw no mapa (a odom de roda gira 90° onde girou 15°, o AMCL puxa de volta). O errinho
  de pose ocasional = os instantes em que o scan-match enfraquecia. **NÃO era "AMCL no giro"** (o
  BO aberto do 30-06) — era a IMU fora.
- **FIX `553e7b3`:** (a) firmware aceita `WHO_MPU6050=0x68` (gyro/accel compatível, mesmas escalas;
  `initMag_` falha gracioso) → **MEGA reflashada, IMU volta a 50 Hz** (confirmado no serial cru:
  frame 0x82 a ~50 Hz); (b) `imu_yaw_sign` `+1.0 → -1.0` (6050 Z p/ baixo). **⏳ FALTA validar na
  bancada:** subir a stack → `yaw_source=imu` + girar p/ ESQ o yaw do /odom SOBE (senão flipa sinal).

### 🗑️ PMW3901 (optical flow) ARRANCADO do robô — TEMPORARIAMENTE SEM FLOW
- O flow estava a 0 Hz (chip-id lia `0xFF` = MISO morto; level-shifter MOSFET marginal, dor
  crônica). Diagnóstico isolou o HW, mas o dono **arrancou o sensor** ("fodase") — o robô fica
  **sem PMW3901 por enquanto**. Decisão: o flow é **baixa prioridade** (yaw = IMU; translação reta =
  roda calibrada; posição = LiDAR/AMCL) → não vale brigar com o shifter.
- **Plano futuro:** se quiser flow de volta, comprar um **breakout da Pimoroni** (regulador +
  level-shift onboard, 5V direto, LED próprio) que mata o shifter marginal — plugar limpo. Não urgente.
- ⚠️ Com o sensor fora, `/optical_flow` fica a 0 Hz (esperado); `use_flow` já era ~no-op na prática.
  `flow_stale=true` no `/trekking/health` é NORMAL agora. NÃO tratar como bug.

---

## 🆕 2026-06-30 (tarde) — Viz do mapa: lidar virou OPCIONAL + conserto do unstuck (escape-spin)

> Tudo na `main` e **deployado na Pi** (HEAD `9c4833a`; web é só Flask, robot_nav buildado com
> colcon). dev = origin = Pi. A run da manhã (`acff7f1`) tinha rodado a rota inteira ("magnífica").

### 1. Viz lidar/robô no mapa — usuário: "horrível pra gravar vídeo, descolados"
- **Sintoma:** nuvem do lidar e boneco do robô descolados; nos giros os pontos varrem pra fora do
  mapa; robô parecia 1-2s atrás do lidar.
- **Diagnóstico por CSV** (`controle_web/logs/scan_lag/`, run 13:55, 5543 amostras):
  `tf_fallback = 100%` — o `lookup_transform` no stamp do scan NUNCA acha o TF (o pipeline
  AMCL+pose_estimator atrasa mais que o scan) → caía no fallback "pose de agora" → girava a nuvem
  torta (ω·age, p99 222ms / max 439ms) e jogava os pontos pra fora.
- **❌ deferred-emit (segurar 1 frame) FALHOU em campo: TRAVOU o scan** (quando o TF nem no frame
  seguinte chega, congela e o robô segue sozinho → descola pior). **Revertido** (`345eba4`→`81eaf32`).
- **✅ Solução aceita:** lidar virou **camada OPCIONAL** (botão `📡 Lidar`, igual o costmap),
  **default OFF** → vídeo limpo (`9742cea`). Desligado, `_on_scan` retorna cedo (poupa CPU/rede).
- **🔴 ABERTO (separado da viz): pose/yaw do robô ERRADA (AMCL).** Robô físico torto mas o mapa
  mostra reto, com o LIDAR casando nas paredes (nuvem certa) = localização, não desenho. Atacar
  depois (provável odometria/AMCL no giro).

### 2. Unstuck — escape-spin (`acff7f1`) estava BURRO em campo: girava em loop, lado errado
- Campo: girou ~5× no lugar, **nunca mais deu ré**, e **pro lado errado**.
- **2 bugs do `acff7f1`:** (a) `_escape_spin_side` flipava pro OUTRO lado no 2º giro no mesmo ponto
  → ia contra o plano; (b) não zerava `move_history` → as 3 translações velhas ficavam no contador
  (120s) e o giro RE-DISPARAVA a cada ciclo pra sempre.
- **✅ Fix `9c4833a`:** (a) **SEMPRE o lado do plano** (tirei o flip + o `escape_spin_history`
  morto); (b) **zera `move_history` ao girar** → o ciclo seguinte cai na ré/avanço antes de girar
  de novo → alterna giro↔translação. 100 testes passam. **⏳ FALTA VALIDAR CAMPO** (deployado, mas
  a stack que estava rodando era a antiga; próximo launch pega o fix).
- ⚠️ NÃO reverter `acff7f1` (usuário recusou) — o ponto era deixar a recuperação esperta, não tirar.

### Nota cosmética (não é BO)
Erro `RTPS_TRANSPORT_SHM ... fastrtps_port7010 open_and_lock_file failed` no boot = locks zumbis em
`/dev/shm` da queda de energia + vários restarts (todos dono `robo`, /dev/shm 1% cheio). Inofensivo
(DDS cai pro UDP). Limpa com reboot (tmpfs) ou `rm -f /dev/shm/fastrtps_*` com a stack DESLIGADA.

---

## 🆕 2026-06-28 (noite) — Unstuck: delay resolvido + avanço adaptativo + giro seguro/último-recurso

> Sessão longa com o dono testando o `sala_grande` (canto cilindro+parede). Tudo **commitado
> na `main`, SEM push ainda** (deploy na Pi quando o dono quiser). 6 commits: `5757791` (raio),
> `eb87f4a` (avanço adaptativo), `7626585` (giro seguro + escape reverse + último-recurso),
> `78a6b56` (aperto-lateral dispara rápido), `9d413db` (removeu o DBG temporário) + docs.
> `forward_speed 0.22` entrou junto no `7626585`. **DBG `recov` REMOVIDO** (já provou as causas);
> ficou só o log conciso `unstuck: X -> Y`.

### 📊 GANHO MEDIDO (2 runs dos mesmos 5 pontos, antes×depois do aperto-lateral)
O delay que ainda irritava (`near_mapped` perde paredes que o LiDAR vê de lado, por offset de
registro AMCL↔mapa → caía nos 15s cautelosos num mapa conhecido) foi atacado com o **aperto
lateral (`side_clear`, do LiDAR) como gatilho rápido** (`78a6b56`). Comparação:

| métrica | baseline | com fix | Δ |
|---|---|---|---|
| duração da run | 1665s (27.8min) | 810s (13.5min) | **−51%** |
| unstuck interveio | 74× | 32× | −57% |
| tempo manobrando | 283s | 61s | −78% |
| delay médio antes de disparar | 3.2s | 1.8s | −46% |
| soma das esperas | 238s | 56s | −76% |
| **fires lentos (≥10s)** | **7×** | **0** | −100% |

Os fires lentos foram a ZERO (alvo direto). Bônus: muito menos travamentos no total (agir em ~2s
em vez de ~14s tira o robô do aperto antes da cascata). Parte do −51% de duração é variância do
nav2, mas as métricas diretas do fix (delay médio, fires lentos) são inequívocas. CSVs/baseline
salvos no scratchpad da sessão (efêmero).

### Os 3 pedidos do dono (06-28 fim do dia) — TODOS atacados
1. **🔴 Delay de ~10-15s pra desencalhar do "conhecido" — RESOLVIDO** (`5757791`). Causa provada
   por log: `mapped_near_radius=0.35` pequeno demais; o robô encosta na parede e o ponto MAPEADO
   dela lê a **~0.54m do centro** (meia-diagonal do chassi ~0.25 + offset de registro pose↔mapa
   ~0.2). Fora dos 0.35 → `near_mapped=False` → caía no caminho cauteloso de 15s. **Fix: raio
   0.35 → 0.6.** Validado: 3/3 desencalharam em ~3s (eram 15s). Seguro (manobras seguem gap-gated;
   cautela dos 15s preservada em espaço aberto, `wall>0.6`). Dono adorou.
2. **🟠 Avanço adaptativo (não reta fixa) — FEITO** (`eb87f4a`). Nova fn `side_clearance` mede o
   aperto LATERAL (o que prende no batente, já que a frente fica livre). Após o nudge mínimo
   (0.20), se havia pinch, CONTINUA até a folga lateral ABRIR (`side_open_delta`) com teto
   `forward_distance_max=0.6`, gap-gated. Validado: avanços ~0.30m (eram 0.20 fixo). Descoberto
   que o avanço RASTEJAVA (0.15 = zona-morta linear do sim) → `forward_speed 0.15→0.22`.
3. **Giro / "ele tem momentos que ia ser muito melhor mas não faz" — RESOLVIDO em 4 iterações**
   (`7626585`), foi o grosso da sessão (whack-a-mole no mesmo canto):
   - **BATIDA:** o giro da escalação varria a quina numa parede a 0.34m (point-turn varre círculo
     ~0.25m + slip). **Fix: GATE** — só gira se `nearest ≥ spin_clear (0.40)`; aborta no meio se a
     folga cair.
   - **LIVELOCK:** o gate bloqueou o giro no canal apertado → oscilava advance↔reverse sem fim.
     **Fix: ESCAPE REVERSE** — na escalação, recua mais fundo pelo rear aberto
     (`reverse_distance_max=1.2`, gap-gated) até achar folga pra girar / sair do canal.
   - **GIRO PREEMPTANDO TUDO:** tentei "giro preferido quando frente bloqueada + folga" → ele
     **girava parado sem fim** mesmo com a traseira aberta (`vao_re=8.4`), atrapalhando o nav.
     **Fix: GIRO = ÚLTIMO RECURSO** — só quando ENCURRALADO (sem ré nem avanço) + folga lateral.
     Prioriza ir reto/onde o nav quer.
   - **LADO ERRADO:** girava pelo `freer_side` (só vê setores frontais ±20-90°) e rodava o rabo na
     parede (obstáculo a -126° traseira). **Fix: DIREÇÃO** — gira PRA LONGE do obstáculo mais
     próximo (usa `near_deg`); fallback no freer_side se o obstáculo está ~reto à frente.

**Resultado final medido (dono testou ida+volta):** spin caiu de 8→1, o robô **SAI do canto
sozinho nos 2 sentidos**, unstuck fica quieto a maior parte do tempo (gaps de ~36s = nav2
dirigindo) e **assiste em vez de atrapalhar**. Dono: "ele chegou". **78 testes** (vários novos) +
**smoke-test do nó** OK (rotina nova: subir o nó 5s e conferir que não crasha — pega bug de
`self.X` não setado que os testes unitários não pegam).

### Pendências desta linha
- ✅ **DBG `recov` removido** (`9d413db`) — já provou as causas. Sobrou só o log `unstuck: X -> Y`.
- ✅ **Unstuck lê `/scan_safe`** (`78426de`) — remap no launch. Tira os fantasmas <0.15m do LD06 das
  leituras do unstuck (near_r/side_clear/gaps). No sim é no-op (laser limpo); protege o real.
- **Validar no REAL** (tudo foi no sim `sala_grande`): o `spin_clear=0.40`, o gatilho por aperto-lateral
  e o `reverse_distance_max=1.2`. Com o `/scan_safe` já no lugar, os fantasmas não devem mais atrapalhar,
  mas confirmar o comportamento real.
- Push pra Pi quando o dono quiser (`git push` + na Pi `git pull`/`reset --hard` + `colcon build robot_nav`
  se não for symlink-install).

---

## 🆕 2026-06-28 — Tuning do path_follower + unstuck "inteligente" (MELHOROU MUITO) + crash corrigido

> Sessão com o dono testando no maze `sala_grande`. Tudo commitado/pushado na `main`.

### path_follower — voltou ao VALIDADO
- Tentei `lookahead 0.4` e depois a "mira-no-canto" (RDP+segment_aim): **ambos pioraram**
  (0.4 = hunting na boca da porta; mira-longe = não segura a linha com a assimetria do skid →
  oscila sem avançar). **Revertido p/ carrot `lookahead 0.6` (validado, porta real 4/4).**
- `forward_speed 0.25 → 0.30` (a pedido — robô tava lento; teto do nav `max_vel_x`=0.35).
- **Lição:** o problema NÃO era o follower — era o **maze apertado demais**. Afrouxei o
  `sala_grande`: portas **0.93 → 1.2 m**, pinch → 1.4 m (mundo+mapa regenerados via
  scratchpad `make_sala_grande.py` + `world2map.py`).

### unstuck — repensado, ficou MUITO melhor (4 mudanças)
Estava atrapalhando (ré/spin no meio das manobras do path_follower). Evoluído p/ "inteligente":
1. **Giro conta como progresso** (`stuck_yaw 0.15 rad`): point-turn legítimo não vira "travado"
   → não dá mais ré no meio de um giro. `[e59775e]`
2. **Opção A — "vai bater de verdade?"**: com a FRENTE LIVRE a parada não é obstáculo →
   **DEFERE** a recovery (dá tempo pro nav) em vez de reverter. `[a176f88]` Mas não pode
   suprimir pra sempre (senão fica preso em bloqueio lateral) → **defere até `front_clear_timeout`
   (15s) e depois age.** `[6d1da6f]`
3. **Direção pela cena**: ao agir, **frente livre → AVANÇA** (passa o batente); **frente
   bloqueada → ré**. Antes dava ré com a frente livre = loop de ré. `[1dbe288]`
4. **Filtro "conheço esse obstáculo?"**: parede MAPEADA perto do robô (`mapped_near_radius
   0.35m`, qualquer lado) → age **rápido (~3s, `front_clear_timeout_mapped`)**; desconhecido
   (pode ser pessoa) → cauteloso (15s). `[a460e24]`

**Resultado medido no log (freeze_capture):** ré caiu de **4% → 0.4%** (10× menos), 43%
andando, **107 m** percorridos numa rodada. Dono: "melhorou um absurdo a velocidade".

### 🔴 CRASH corrigido (fim da sessão) — ATENÇÃO amanhã
- O filtro #4 introduziu um crash: o `_tick` usava `self.mapped_near_radius` mas eu só setei
  o param em `self.cfg`, **não como atributo do nó** → `AttributeError` FATAL no timer → o nó
  **morria** (exit 1) = "parou o unstuck todo". **CORRIGIDO** `[c3632b3]` (setado
  `self.mapped_near_radius = g[...]`). Nó sobe sem crash, 67 testes unstuck.
- ⚠️ **Lição p/ amanhã:** os **testes unitários NÃO pegam bugs do `main()`/`_tick` do nó**
  (só testam a classe `UnstuckSupervisor` pura). Qualquer mexida no nó precisa de um
  **smoke-test do nó** (`ros2 run robot_nav unstuck_supervisor` por uns segundos) antes de
  confiar. Vários params do nó são `self.X` (não `self.cfg.X`) — fácil esquecer de setar.

### 🎯 PRA AMANHÃ — melhorar o unstuck (FEEDBACK DO DONO 06-28, fim do dia)
> Veredito do dono: "**melhorou um absurdo**", a parada (defer) do unstuck está ajudando
> MUITO. Mas 3 coisas concretas pra atacar:

1. **🔴 AINDA DEMORA ~10s pra andar pra frente no unstuck.** O fast-path do "conhecido"
   (`front_clear_timeout_mapped` ~3s) **não está cortando o delay** — caiu pros ~10s do
   `stuck_timeout`. **Investigar PRIMEIRO:** o `near_mapped` está disparando? (a) o teste do
   dono pode ter sido com o nó CRASHADO (antes do `c3632b3`) → re-testar com o fix; (b) o
   obstáculo que trava (batente) pode não estar caindo no raio `mapped_near_radius` (0.35m) →
   subir o raio, ou logar `near_mapped`/`DBG recov` pra ver; (c) talvez baixar o
   `front_clear_timeout` geral. **Meta: tirar o delay → o dono disse que "se tirar esse delay
   entre um unstuck e outro ele melhora bastante".**
2. **🔴 TIRAR O DELAY ENTRE UM UNSTUCK E OUTRO.** Tem um `grace` (2.0s) entre manobras —
   encadear unstucks consecutivos mais rápido (baixar/zerar o grace, ou re-armar na hora se
   ainda travado). É o que mais incomoda na fluidez.
3. **🟠 AVANÇO ADAPTATIVO (não reta hardcode).** Hoje o avanço é `forward_distance` fixo
   (0.20m). O dono quer que ele **ande o SUFICIENTE pra SAIR do obstáculo que o travava**, não
   uma reta fixa. Ideia: avançar até a cena LIBERAR (ex.: até o `front_gap`/lateral abrir, ou
   até passar o ponto de contato `front_block_point`), com teto de segurança. Mesma ideia
   pode valer pra ré.

### Outras pendências de método/segurança
4. **Smoke-test do nó:** os testes unitários NÃO pegam bug do `main()`/`_tick` (só a classe
   pura `UnstuckSupervisor`) — foi o que deixou o crash do `mapped_near_radius` passar.
   Considerar um teste que sobe o nó com /scan+/odom fake e confirma que não crasha.
5. Teto de repetição no "avança quando frente livre" (não empurrar pra sempre se não liberar).
6. **Conferir o crash de fato sumiu no sim** (relançar `./launch.sh --sim --nav2
   --world=worlds/sala_grande.sdf --map=maps/sala_grande.yaml`) — o dono testou parte com o
   nó possivelmente já morto.

---

## 🆕 2026-06-27 — Consolidação na main + L reprovado + maze de teste + BOs novos do path_follower

### Git: TUDO consolidado na `main`
- A branch `feat/reto-mais-point-turn` (path_follower validado + 2-mux + porta nativa + 24 commits)
  foi **fast-forward pra `main`** (`6bc8dea`→`5ad05d6`, sem conflito) e **pushada** pro origin.
  `main == origin/main == branch`. ⚠️ Próximo `git reset --hard origin/main` na Pi traz mudança
  GRANDE de uma vez (path_follower no lugar do fluxo antigo) — já validado em campo, mas testar curto.

### CONTORNO-EM-L: A, B e C TODAS REPROVADAS → revertido, mantido Theta* diagonal
- **A** (Theta* how_many_corners 8→4): line-of-sight costura diagonal, 4≈8. ❌
- **C** (SmacPlanner2D): 8-conn fixo (Moore), sem 4-conn → escada de 45°. ❌
- **B** (nó `plan_manhattanizer`: pós-processa o /plan em pernas ortogonais com RDP+checagem de
  costmap): **forma certa** no teste estático (2 cantos de 90° limpos), MAS **ao DIRIGIR ficou pior**
  (para-gira-anda seco) — dono: "tá uma merda, o anterior tava bem melhor". **REVERTIDO** (nó/testes/
  wiring/entry removidos). ❌
- **Lição:** nenhum jeito de forçar o "L" melhorou; a **diagonal suave do Theta* + path_follower é o
  melhor que temos**. NÃO re-tentar A/B/C. Commit `5ad05d6` + spec `2026-06-26-contorno-em-L-design.md`
  têm o detalhe. **nav2 funcionalmente IDÊNTICO ao de antes** (provado: diff e548395→HEAD = só comentário).

### Assets de SIM novos (untracked, NÃO commitados)
- `worlds/sala_grande.sdf` + `maps/sala_grande.{pgm,yaml}` — **maze 16×10 DIFÍCIL**: serpentina de 3
  portas 0.93m + 2 chicanes + pinch de 0.75m + 9 obstáculos + dead-end decoy. Mapa gerado do SDF
  (frame==mundo, localiza de cara). Rodar: `./launch.sh --sim --nav2 --world=worlds/sala_grande.sdf
  --map=maps/sala_grande.yaml`. Plano start→goal = 23.3m (resolvível).
- `worlds/educacao_criativa.sdf` + `worlds/meshes/` (6.6MB) — **recuperado** do `7ce1cac^` (foi removido
  pra economizar espaço). `launch.sh` ganhou `GZ_SIM_RESOURCE_PATH=worlds/` (senão mesh não resolve).
- `maps/sim_sala.*` — mapa SLAM do `sala.sdf` que fiz no tour autônomo (frame alinhado ao mundo).
- Tools em scratchpad (NÃO no repo): `map2world.py` (occupancy→SDF), `world2map.py` (SDF→occupancy).
- ⚠️ **Mapa real perdido no dev:** o `golden` (06-10) é o único mapa real no git; o `sala.*` local é
  uma caixa 6×6 VAZIA (sobrescrita). O último SLAM bom do robô (melhor que golden) está SÓ na Pi —
  puxar quando a Pi voltar (`ssh robo@robo-desktop.local`, offline agora).

### 🔴 BOs NOVOS — path_follower no maze apertado (dono testando sala_grande 06-27, NÃO atacados)
Reproduzidos no `sala_grande`. O nav2 não mudou — apareceram porque o maze é apertado vs inflação 0.45.
1. **🔴 Robô vira ANTES da linha do plano e EM CIMA do batente** — atravessando a porta, ele corta o
   canto cedo: o `/plan` foge certo da parede/batente, mas o robô vira sobre o batente, **entra no
   costmap**, e o collision_monitor PARA ele dentro da porta. Devia **sair TODO de entre os batentes
   antes de virar**, aí girar tranquilo. **FIX pedido pelo dono: deixar o robô mais RÍGIDO em seguir
   os limites do costmap / a linha do plano** (não cortar canto, não virar cedo). Mexer no
   `path_follower` (lookahead/carrot, histerese de giro, ou condicionar o giro a estar fora da zona
   de inflação). Robô também **lento demais**.
2. **🔴 unstuck ativando DO NADA** — o robô está **girando** e começa a **dar ré do unstuck** sem
   motivo, fodendo o nav2. Investigar `unstuck_supervisor` (provável: confunde giro-no-lugar com
   "travado" / lê front_gap errado / dispara recovery durante manobra legítima). Já tinha 2 bugs
   conhecidos (lê /scan cru não /scan_safe; só conhece goal_active não distância) — ver §2 abaixo.

**PRÓXIMO PASSO (retomar aqui):** atacar BO#1 (path_follower mais rígido, não cortar canto/virar dentro
do batente — 1 mudança por vez) e BO#2 (unstuck disparando no giro). Opcional: afrouxar o sala_grande
(portas→1.2m, pinch→≥1.0m) se quiser separar "robô" de "maze apertado". Validar no sim (`sala_grande`),
depois no real. NÃO mexer no tuning do nav2 que é o do robô real sem necessidade.

---

## 🆕 2026-06-26 — MARCO: path_follower no real + Nav2 ATRAVESSA A PORTA SOZINHO

**Git:** trabalho na branch **`feat/reto-mais-point-turn`** (HEAD `2ca7e96`), **NÃO mergeada na
main** (a main nem tem o `path_follower`). Deployada na Pi (`git fetch && git reset --hard
origin/feat/reto-mais-point-turn` → `colcon build robot_nav`; web entra no relançamento).

### 🤖 Sessão autônoma 06-26 (eu sozinho, dono ausente) — resumo
Trabalhei a lista de BOs de software/sim (hardware ficou pro dono). Tudo na branch
`feat/reto-mais-point-turn`, commitado/pushado, **validado no sim onde deu**:
1. ✅ **2-mux** (collision protege TODA a autonomia, sem SPOF) + **bond_timeout 4→20s** — validado no
   sim (anda+collision freia o seguidor+unstuck fura). [`25d12e9`,`2091635`,`7c6d9a0`]
2. ✅ **Costmap web intermitente** → service call `get_costmap` (entrega garantida) — validado ao
   vivo. [`edffefa`]
3. ✅ **Zona-morta linear no sim** (`sim_actuator_model`) — 7 testes. [`7bf8c8b`]
4. 🟡 **Boneco atrasado (scan_lag)** — hipótese websocket DERRUBADA (está no venv); consertei o bug
   `age_ms` (sim-time) + criei `measure_web_lag.py`; **causa raiz NÃO fechada** (server off + robô
   off → sem dado ao vivo). [`d8c04ad`]
5. ⏸️ **#2 porta SLAM** — NÃO mexi (mapa ativo + ambíguo; `sala.pgm` parece sim que sobrescreveu o
   real). Deixei pra você. [`d7adb30`]
6. ⏸️ **Contorno em "L"** — NÃO implementei (driver validado, checkpoint prometido). Spec de decisão
   pronto: `docs/.../2026-06-26-contorno-em-L-design.md` (recomendo opção A→C).

**Pendências que precisam de VOCÊ / hardware:** validar 2-mux + min_speed=0.22 + costmap no REAL;
rodar `measure_web_lag` com server no ar p/ fechar o scan_lag; decidir o #2-porta (mapa real perdido?);
escolher a abordagem do "L". Detalhes em cada BO abaixo (seção 2) e nos "Próximos passos" (seção 4).

### 🏆 Marco maior
- **`path_follower` VALIDADO no real** — seguidor reto+giro-no-lugar que segue o `/plan` do
  Theta\* e ignora o tracking do controller_server (publica `follow_vel`, prio 15). Dono:
  **"visivelmente melhor" e "igual ao sim, chega a ser engraçado".** Sim≈real provado.
- **🚪 Nav2/path_follower ATRAVESSA A PORTA NATIVAMENTE — 4/4 no real**, com a porta DELETADA
  do mapa, sem ponto pré-porta, nos 2 sentidos (inclusive do ângulo que antes dava "merda
  total"). O DWB velho não threadava o vão (era POR ISSO que o `door_crossing` existia); o
  seguidor vai reto pelo vão + giro decidido = threada sozinho. **→ `door_crossing` virou
  OBSOLETO e foi DESATIVADO** (comentado no `nav2.launch.py`; re-habilitar = descomentar +
  colcon). Bug do arme dele (caçado à toa): provado por log que `goal_succeeded` do
  `navigate_through_poses` nunca dispara no ponto intermediário → `cleared=False` sempre → não
  arma. MOOT agora. Meus fixes de pré-porta (busca 2D, zone cap 1.0, folga 0.50, fallback no
  mais livre) ficaram inertes — REVER se religar o door.

### Mudanças aplicadas (commits 69bc9ac → 2ca7e96)
- ~~collision_monitor filtra o seguidor~~ **REVERTIDO (`7a6de77`) → RESOLVIDO de vez com 2-MUX
  (2026-06-26, ⏳ validar):** o revert virou band-aid. Causa raiz do SPOF era o **bringup flaky**.
  Dois passos:
  - **Passo 1 (`25d12e9`): `bond_timeout` do `lifecycle_manager_navigation` 4.0 → 20.0s.** A Pi
    lenta demorava >4s pra confirmar o bond do `velocity_smoother` → o lifecycle derrubava a stack
    INTEIRA no meio (collision às vezes nem ativava → nav subia pela metade, "parecia bug"). Agora
    bringup atômico/confiável.
  - **Passo 2 (2-MUX): collision protege TODA a autonomia, sem SPOF.** Pipeline novo:
    `smoother(nav_vel)/path_follower(follow_vel)/door(door_vel)` → **`twist_mux_auto`** →
    `auto_vel_raw` → **`collision_monitor`** → `auto_vel` → **twist_mux FINAL** (prio 10) →
    `cmd_vel`. O **unstuck (30) e o humano (web/PS4)** entram no mux FINAL, A JUSANTE do collision
    → seguem furando (resgate/override sempre funcionam). **Collision agora é OBRIGATÓRIO:** sem
    ele, `auto_vel` some e a autonomia não anda (mas o humano dirige). Antes só `nav_vel` era
    filtrado e o seguidor (driver atual) furava → **buraco de segurança fechado.** Arquivos:
    novo `config/twist_mux_auto.yaml`, `twist_mux.yaml` (agora `autonomy`/auto_vel + unstuck +
    humano), `nav2_params*.yaml` (collision in/out = auto_vel_raw/auto_vel), `nav2.launch.py`
    (smoother→nav_vel + nó twist_mux_auto), `unstuck`/`door` (tap `nav_vel_raw`→`nav_vel`, rename
    puro), `freeze_capture` (loga auto_vel_raw/auto_vel). 166 testes ✅. Plano:
    `goofy-kindling-hopcroft`. Commits `25d12e9` (bond) + `2091635` (2-mux) + `7c6d9a0` (fix).
  - **✅ VALIDADO NO SIM (2026-06-26, dev):** anda sob nav, o collision FREIA o seguidor (antes
    furava), e o unstuck ainda fura o collision. ⏳ **FALTA validar no real.**
  - 🐞 **Bug pego no sim (corrigido `7c6d9a0`):** o `twist_mux_auto.yaml` tinha a chave de topo
    `twist_mux:` mas o nó chama `twist_mux_auto` → o ROS casa params pelo NOME DO NÓ → subiu com
    DEFAULTS (não assinava os vels + publicava TwistStamped que o collision não consome) → a nav
    morreu igual ao revert. **Lição: chave do YAML = nome do nó; testes unitários NÃO pegam isso.**
- **local costmap inflation 0.25 → 0.35**; **global mantido 0.45** (folga de obstáculo).
- **w_traversal_cost do Theta\*: testei 2.0→0.5 (menos contorno), REPROVADO** (enfiava o plano
  em vão IMPOSSÍVEL parede-obstáculo) → revertido 2.0. Lição: w_traversal só troca "volta larga"
  por "buraco impossível", NUNCA vira a "L" (reta→canto→reta) que o dono quer — Theta\* é
  any-angle de menor distância (corta diagonal). **Fix do contorno = simplificador no
  path_follower (reusar plano seguro do Nav2 e dirigir em retas) — TODO, NÃO feito.**
- **Web:** overlay opcional do `/global_costmap` no mapa (botão 🗺️ Costmap, PNG RGBA translúcido).

### Regressões achadas + corrigidas
- **Pose inicial (commit 57c8b13 quebrou):** `set_initial_pose` no launch tinha default `false` →
  no REAL o AMCL nascia NÃO-localizado (antes auto-localizava em (0,0,0) pelo yaml). E SEM pose
  o ponto pré-porta nem saía. Fix: default `'true'`. Sim ainda passa spawn explícito.
- **🔴 NÃO FINALIZA OS PONTOS (resolvido) — era ZONA-MORTA LINEAR:** o robô chegava ~0.17 m do
  goal e CONGELAVA (`vx=0.11 wz=0`, pose travada) — não girava pra finalizar; precisava empurrar
  no controle. Causa: o ramp de aproximação do `path_follower` baixava p/ `min_speed=0.10` ≈ 0.11
  m/s, **abaixo da zona-morta linear do robô pesado** (manda 0.11 e não anda). **Fix: `min_speed
  0.10 → 0.22`** (0.11 trava, 0.25 cruise anda → zona-morta no meio). ⏳ FALTA VALIDAR; se ainda
  rastejar, subir p/ 0.25. **A zona-morta LINEAR nunca foi medida** (só a do giro=1.7) e o
  `sim_actuator_model` só modela o giro → o sim NÃO pegava esse trava.

### BOs novos
- ✅ **Overlay do Costmap global intermitente na web (botão 🗺️) — RESOLVIDO (2026-06-26).** Causa
  raiz (provada): com `always_send_full_costmap: false` (perfil Pi) o `/global_costmap/costmap` sai
  **latched UMA vez** na ativação e a entrega transient_local pra late-join **falha** (testado:
  3/3 não recebe); o web só assinava o grid cheio (ignorava os diffs de `costmap_updates`) → o
  overlay só aparecia se o web estivesse assinado no instante do one-shot → intermitente pela ordem
  de boot web×nav. No sim PURO funcionava porque o perfil default usa `true` (republica sempre).
  **Fix:** trocado a subscription frágil por **service call `get_costmap`** sob demanda ao ligar a
  camada (request/response = entrega garantida; mapa global é estático → busca única + cache).
  Conversor novo `Costmap(0..255)→OccupancyGrid` reusa a conversão PNG testada. Front-end inalterado.
  `controle_web/map_service.py` + testes (8 ✓). **Validado ao vivo no sim** (get_costmap 160×120 +
  overlay funcional). ⏳ Falta validar no real (mesmo caminho).
- ✅ **sim modela zona-morta LINEAR (2026-06-26):** o `sim_actuator_model` agora aplica zona-morta
  no `linear.x` (param `linear_deadzone`, default 0.15 — entre o 0.11 que trava e o 0.25 que anda;
  nunca medida). Lógica extraída em funções puras `model_linear`/`model_theta` + 7 testes. Agora o
  sim reproduz o "congela no goal" por comando linear pequeno. ⏳ medir o limiar real algum dia.

### ⏭️ Próximo
1. **Validar o `min_speed=0.22`** (finaliza os pontos sem empurrão?).
2. Validar travessia da porta SEM door em mais cenários (já 4/4).
3. **Contorno em "L"** (reta→canto→reta) — ⚠️ **DECISÃO PENDENTE DO DONO** (spec
   `docs/superpowers/specs/2026-06-26-contorno-em-L-design.md`). Sessão autônoma 06-26 mapeou: NÃO é
   "só um simplificador" (Theta* é any-angle → o contorno já É uma diagonal; juntar colineares
   mantém a diagonal). Pra virar "L" muda a ROTA (axis-aligned). Opções: **(A)** `how_many_corners
   8→4` no Theta* (1 linha, testar primeiro, talvez o LOS ainda corte); **(C)** trocar planner p/
   grid-A*/Smac 2D 4-conn (fix estrutural, não toca o driver validado); **(B)** Manhattan-izar no
   path_follower (evitar — precisa costmap, risco no driver). Recomendação: A→C. NÃO implementei
   (checkpoint prometido antes de tocar o driver ativo).
4. Reativar/revalidar o costmap na web; modelar zona-morta linear no sim.

---

## 0. Onde estamos (git)

- Branch de trabalho agora: **`feat/reto-mais-point-turn`** (HEAD `2ca7e96`, deployada na Pi) —
  ver a seção 🆕 2026-06-26 no topo. **NÃO mergeada na main ainda** (validar mais antes). A main
  tem o estado anterior (sem `path_follower`).
- ~~Branch de trabalho: `main`~~ (era a decisão até 06-24; o trabalho do path_follower abriu a
  branch nova e ainda não voltou pra main).
- A branch `feat/door-para-pra-pessoa` foi merjada na main (PR #1 no GitHub `feb1be9`),
  e os 7 commits que ficaram de fora do PR (sim 4-rodas + diagnóstico de scan-lag) foram
  trazidos pra main no merge `686c57f`.
- **Fluxo de deploy na Pi:** editar no dev → commit → push → na Pi
  `git fetch && git reset --hard origin/main` → `colcon build` (do pacote alterado, ex. `robot_nav`).
  Acesso: `ssh robo@robo-desktop.local` (a Pi troca de IP toda hora; usar `robo-desktop.local`
  e fazer retry até conectar). ROS = **jazzy**.
- Pi deployada e buildada em `4f8b306` (com o nó `freeze_capture`). Dev/GitHub à frente só com
  docs (README atualizado + `CONEXOES.txt`).

**Arquivos de referência no repo (qualquer PC):**
- `ESTADO_PROJETO.md` (este) — estado vivo: BOs, avanços, TODO.
- `CONEXOES.txt` — pinagem MEGA + cabo hoverboard + USB da Pi (fonte = firmware).
- `README.md` — guia completo (sim/real, setup, modos, tuning).

---

## 1. Estratégia SIMULADOR vs REAL (decisão 2026-06-24)

**Problema:** o robô real vive ficando sem bateria e a gente fica parado esperando carregar.
**Plano:** desenvolver/iterar no **simulador** e soltar o **real só pra validar**.

### O simulador roda o MESMO nav do robô real?
**O cérebro sim, a física não (ainda).**

- O `sim.launch.py` só sobe o **Gazebo (gz Harmonic) + robô simulado + twist_mux**.
  O resto (nav2/slam) é lançado pelo **mesmo `launch.sh`**, **mesmos nós**:
  `nav2.launch.py` (planner, controller/DWB+RotationShim, **door_crossing**,
  **unstuck_supervisor**, **scan_sanitizer**, costmaps) e `slam.launch.py`.
  → Toda a **lógica de navegação é idêntica** à do real.
- **Diferenças que importam:**
  1. **Parâmetros:** real roda com `--pi` → usa `nav2_params_pi.yaml` (perfil leve da Pi).
     O sim no dev, sem `--pi`, usa `nav2_params.yaml` (default). Tuning pode divergir.
     → Pra fidelidade, rodar o sim **com `--pi`** ou comparar os dois YAMLs.
  2. **Camada física/sensores:** mega_bridge, hoverboards, LiDAR LD06, IMU MPU6050 e flow
     PMW3901 são **substituídos por plugins do Gazebo** (DiffDrive, lidar, odom). Logo, o sim
     **NÃO reproduz** por padrão: ruído/confiança dos sensores, patinagem do skid-steer,
     EMI do motor, travas de I²C da MEGA, quedas de BMS, lag de transporte do scan, etc.

### 🎯 ANÁLISE DE LACUNAS sim vs real (06-24) — fechar do mais impactante pro menos
Decisão do dono: **deixar tudo igual** (sim = real), 1 gap por vez.

| # | Gap | Impacto | Esforço | Status |
|---|-----|---------|---------|--------|
| 1 | **Config Nav2 era OUTRA** — sim usava `nav2_params.yaml` (DWB puro, sem RotationShim, max_vel_theta 0.8); real usa `nav2_params_pi.yaml` (RotationShim, theta 6.0, /scan_safe, obstacle_layer) | 🔴 enorme | trivial | ✅ **FEITO+VALIDADO** — `launch.sh` faz `--sim --nav2` usar `nav2_params_pi.yaml`. **06-24: usuário viu o sim "burro IGUAL ao real" — mesmo código, mesma burrice.** |
| 2 | **Zona-morta + assimetria do giro** — real não gira <1,7 rad/s, satura ~2,5 (sim já capa 2,5 ✓, mas SEM zona-morta nem assimetria). Provável causa do "congela perto do goal" | 🔴 alto | médio | ✅ **FEITO** — nó `sim_actuator_model` entre twist_mux e DiffDrive aplica `giro=0.6·(\|cmd\|−1.7)`, satura 2.5, zona-morta 1.7, direita ×1.05. Params tunáveis. |
| 3 | **Odom ideal no sim** (DiffDrive perfeito) vs real (pose_estimator funde roda+IMU+flow, superestima yaw na patinagem). Sim nem roda o pose_estimator | 🟠 alto | grande | ⬜ a fazer |
| 4 | **LiDAR limpo** vs LD06 com fantasmas <0,15m + ruído (os fantasmas que envenenam o `front_gap` do unstuck) | 🟠 médio | médio | ⬜ a fazer |

**🎉 MARCO 06-24:** com #1+#2 o sim já reproduz o robô "burro" do real ("é o mesmo código").
✅ **Mundo com obstáculos criado** (`worlds/sala.sdf`, agora DEFAULT do `--sim`; `empty.sdf` =
template vazio): sala 8×6 dividida por uma parede com **porta de 0,93 m** (igual à real) +
caixas/cilindro. Robô spawna em (0,0) encarando a porta. Pronto pra reproduzir "vira cedo na
parede / congela perto do goal / travessia de porta" e atacar com o `freeze_capture` (CSV local,
sem ssh). Régua: o que funcionar no sim **valida no real** em janela curta de bateria.
Dados reais medidos pra calibrar o sim: IMU ~99%; giro ≈ `0,6·(cmd−1,7)`, satura ~2,5, não gira
<1,7, direita gira mais (3% a 4–6 rad/s, 30% a 2 rad/s); odom de roda superestima yaw; flow cospe
lixo na EMI.

> ⚠️ `./launch.sh --sim` completo precisa `sudo apt install ros-jazzy-twist-mux` (faltava na dev).
> Mundo atual = sala-caixa 6×6 (`empty.sdf` customizado). Geometria do robô já é a REAL
> (chassi 0.37×0.35, 4 rodas skid-steer via DiffDrive 2+2 joints, LiDAR no topo).
> Sim validado local no gz Harmonic: anda / gira / lidar OK. `mu2=0.4` é o knob do giro.

---

## 2. BOs ABERTOS (problemas conhecidos)

### Físicos / hardware
- **Bateria acaba rápido** → trava os avanços de campo (motivador da estratégia de sim).
- **BMS do hoverboard desarma** sob stall/rotor bloqueado (39V→6V); botão de emergência reseta.
  Monitor de tensão (CSV 10Hz + chip na UI) já implantado na Pi — **falta ler o CSV no próximo desarme**.
  ⚠️ **06-24: desarmou DE NOVO "do nada" e não voltava** → tive que desligar no meio do teste do
  congelamento; bateria foi pra carga. (mais um caso pro CSV do power_monitor).
- **MEGA trava o firmware no I²C** sob EMI (já mitigado: `Wire.setWireTimeout` + watchdog WDTO_2S
  no firmware + guarda `wheel_fresh` no Python). Validado, mas monitorar.

### Navegação / software
- **🟢 ATUALIZAÇÃO 06-26 — o congelamento perto do goal com o `path_follower` era ZONA-MORTA
  LINEAR** (`min_speed=0.10` ≈ 0.11 m/s, abaixo do limiar do robô pesado) → fix `min_speed 0.10
  → 0.22` (ver seção 🆕 no topo, ⏳ validar). O abaixo é a investigação 06-24 na era DWB/unstuck
  (outro controlador) — manter como histórico; a raiz pode ser diferente entre os dois.
- **🔴 ATIVO — robô CONGELA perto do goal (investigando 06-24):** ele para pertíssimo do ponto,
  dá ré do unstuck, volta, repete (não é 100% das vezes). **Causa raiz = ele NÃO se mexe sob o
  nav** (nas janelas de `monitoring` a pose não muda: `1.99,-0.24`→`1.99,-0.24`). O unstuck é
  só **agravante** (empurra ele pra lá e pra cá), não a origem. Dois bugs confirmados no
  `unstuck_supervisor`: (1) lê o `/scan` CRU, não `/scan_safe` → `front_gap` pega fantasma <0,15m
  (pisca `0.10↔2.72`) → escolhe ré em vez de cutucar pra frente; (2) só conhece `goal_active`,
  não a distância ao goal → dispara recovery durante a aproximação final legítima.
  **Falta provar o PORQUÊ do congelamento:** nav comanda giro e odom≈0 (zona-morta/collision
  congela) OU nav comanda ~0 (ponto inalcançável/colado em parede). Coletor `freeze_capture`
  (nó no `nav2.launch.py`, grava `controle_web/logs/freeze_capture.csv` a cadeia
  cmd_vel_nav/nav_vel/cmd_vel/odom) **deployado na Pi `4f8b306` mas NÃO validado — bateria do
  hover cortou antes de reproduzir.** Próxima sessão real: religar → `./launch.sh --nav2` →
  reproduzir o congelamento → eu leio o CSV.
  **06-24 (sim): `freeze_capture` turbinado** — além da cadeia de velocidade, grava
  `freeze_diag.csv` (5 Hz): heading do robô (TF map→base_link) × direção do `/plan` a 0.5 m
  (`plan_rel_deg`) × obstáculo à frente (`/scan`, ±15°) + último `cmd_vel_nav`. Prova
  "planner manda contornar X° e o robô aponta reto na parede". Achado parcial no sim: andando,
  o DWB só pede giro <0.55 rad/s (zona-morta mata) → reto; giro forte só parado (point-turn).

  **🔬 RODADA AUTÔNOMA NO SIM 06-24 (eu sozinho, sem o usuário) — BURRO REPRODUZIDO+CAPTURADO:**
  Mundo `sala.sdf`, mapa `sim_sala` (SLAM autônomo), goal (-2.8,-1.4) atrás do cilindro. Log:
  `controller_server: Failed to make progress` repetido (= assinatura do real). Dado do
  `freeze_diag.csv` (310 amostras, robô engajado) — **CSVs preservados em
  `controle_web/logs/sim_burro_2026-06-24/`**:
  - `plan_rel`: mediana **−24°**, máx **104°** → o planner pede curva forte o tempo todo.
  - **Andando (vx>0.05, 137 amostras): |wz| mediana 0.55 rad/s** (abaixo da zona-morta 1.7 → o
    modelo zera → vai RETO). **Girando forte (|wz|>1.7, 141 amostras): vx≈0** (point-turn parado).
  - Ou seja: **reto OU giro-no-lugar, NUNCA arco coordenado.** Ex. t+18s: plan pede −18°, robô
    `vx=0.35 wz=−0.55` (giro morto) indo num obstáculo a 0.82 m → aproxima, point-turna, dá ré
    (unstuck vx=−0.15), passa do goal (chegou a y=−2.44, goal y=−1.4), não converge → aborta.
  - ✅ **CAVEAT RESOLVIDO COM DADO REAL (06-25, `arc_calib.py`):** o caveat era "será que o
    'não arqueia' é artefato do meu sim_actuator_model?". Esclarecido pelo dono: o sim está
    CERTO — o real nunca arqueou, foi **decisão dele** mandar parar de arquear (arco saía fraco
    demais) e só girar no lugar. Mas faltava PROVAR se o HW é incapaz ou se só não tunamos.
    Rodei o `arc_calib` (mede giro ANDANDO, vx=0,25 fixo, 1 wz por play, lê /odom fundida).
    **wz comandado → efetivo (% do comando):** 0,3→3% | 0,5→3% | 0,8→2% | 1,2→3% (=RETO com
    ruído) | 1,7→7% | **2,5→19% (0,47 rad/s, raio 0,53 m)**. Ele **SEMPRE sub-vira andando**,
    nunca passa de 19%. Andar a 2,5 dá ~0,47 rad/s = IGUAL ao giro PARADO a 2,5 → andar quase
    não muda a autoridade. **VEREDITO: o robô REALMENTE não arqueia — é FÍSICO (diferencial
    pequeno não vence a patinagem lateral do skid-steer), não tuning. DWB é incompatível.**
    Usar SÓ o arco do 2,5 como primitivo foi REJEITADO (tudo-ou-nada, beira da saturação =
    pior assimetria + gatilho do BMS, só 19% fiel). `cmd_vel_to_wheels` confirmado SEM
    zona-morta no SW (cinemática pura) → a zona-morta é firmware+físico. Ver [[feedback_no_arc_turns]].
    Script `f6ebda8`, CSVs `/tmp/arc_calib*.csv` na Pi.
  - **Direção do fix (decidida):** reto no corredor + **giro 90° no lugar** (autoridade alta +
    malha fechada no yaw da IMU, igual ao spin do unstuck). skid-steer com zona-morta não esterça andando →
    DWB (arcos suaves) é incompatível. Opções: (a) baixar a zona-morta no controle/firmware p/
    o arco existir; (b) controlador que faça reto+point-turn deliberado; (c) tunar p/ commitar no
    point-turn cedo. Ver [[feedback_no_arc_turns]] e [[project_nav2_recovery_nao_dispara]].
  - **Pendência do método:** o SLAM autônomo saiu offset/pequeno (só canto inf-esq, x≤1.96 y≤0.64)
    → goal fora dos bounds = planner aborta sem mexer (NÃO confundir com o bug). P/ testar a PORTA
    (sala direita) preciso de mapa cobrindo tudo: melhorar o tour OU gerar o mapa da geometria do SDF.
  - ✅ Instrumento `freeze_diag.csv` (heading×plan×obstáculo) FUNCIONOU — provou a assinatura.
- **NAV2 "burro"** (mesmo tema do congelamento acima): vira cedo/forte, faz curva em tangente
  em vez de ir reto no corredor e girar 90° no lugar → chega de cara/paralelo na parede, precisa
  de 2 rés pra sair. Skid-steer **não faz arco** — realinhar tem que ser **giro no lugar** com
  autoridade alta (~6.0) + malha fechada no yaw da IMU.
- **🟡 Viz do boneco atrasada** na UI web (lag de transporte/socketio). **INVESTIGADO 06-26 (sessão
  autônoma), AINDA ABERTO — precisa de dado ao vivo pra fechar:**
  - ❌ Hipótese "falta `simple-websocket` → preso em long-polling" **DERRUBADA**: está instalado no
    venv (`.venv`, `simple-websocket==1.1.0`) → o upgrade pra websocket PODE ocorrer. (Cuidado: no
    python do SISTEMA falta — só o venv conta.)
  - 🐞 **Bug no próprio diagnóstico (corrigido):** o `age_ms` do `scan_lag.csv` era LIXO no sim — comparava
    o stamp do scan (sim-time, /clock) com `now=time.time()` (wall) → ~1.78e12 ms. Fix: usar
    `self._node.get_clock().now()` (respeita use_sim_time) pra o age. Agora o age é válido em sim E real.
  - 📊 Dos CSVs (sim): emit ~7 Hz (throttle SCAN_PUBLISH_HZ=10), **tf_fallback 71%** (a TF no stamp
    exato do scan quase nunca está no buffer → cai no "latest"; investigar se é artefato de sim-time
    ou desync real de TF). age era inútil (bug acima).
  - 🔧 **Ferramenta nova pronta:** `controle_web/measure_web_lag.py` — cliente socketio que mede o
    atraso REAL de transporte (`recv − _sts`) + qual transporte o engineio negociou + taxa. **Rodar
    com o server no ar:** `controle_web/.venv/bin/python controle_web/measure_web_lag.py`.
  - **PRÓXIMO (precisa server no ar / robô):** rodar o `measure_web_lag` → se a latência for alta e/ou
    crescente (backpressure) = transporte/payload (downsample do scan, taxa, websocket de fato); se
    baixa = é front-end (render/easing no `map.js`). No real, sobre wifi, medir tb. NÃO fechei a causa
    raiz: server estava fora do ar e robô desligado nesta sessão.
- **#2 porta no SLAM** — **NÃO mexi (sessão autônoma 06-26): risco alto + ambíguo, deixei pra você.**
  Inspecionei `maps/sala.pgm` (renderizei): vejo **UMA** porta na divisória (vão único no meio), não
  duas → não localizei o "#2 fantasma" com confiança. ⚠️ **Achado preocupante:** o `sala.pgm`
  atual (06-26 15:53) tem os **obstáculos do SIM baixados** (3 caixas + cilindro do `sim_sala.sdf`)
  e geometria 8×6 → parece um mapa do SIM que pode ter **SOBRESCRITO** o mapa real re-SLAMado de
  06-23; reforça: `sala.posegraph`/`sala.data` ainda são de **Abr-24** (inconsistente com `.pgm`/`.yaml`
  de 06-26), e `sala.doors.json` está vazio. **Não editei** (mapa binário ativo afeta localização +
  não dá pra verificar sem você + talvez seja o mapa errado). **Pra você:** confirmar se o `sala`
  real foi perdido (restaurar do `mapa_golden_*` ou re-SLAM no real) e apontar qual é a "#2 porta".
- **Logs de DEBUG ainda no código** pra remover após validação:
  - `DBG recov:` no unstuck (recovery contextual).
  - ⚠️ Lição: **nunca alternar throttle na mesma chamada de log** — um log DBG meu já matou o
    door_crossing em campo (revertido em `267a00a`).

---

## 3. AVANÇOS recentes (o que já está bom)

- ✅ **DOOR (travessia de porta) RESOLVIDA (06-23):** o robô atravessava ~8° torto e batia o
  batente esquerdo. Causa **não era** a door — era **mapa skewed** (AMCL achatava o yaw real do
  IMU pra casar com parede torta do mapa). **Refazer o SLAM** (salvo como `sala`) consertou →
  travessia perfeita. Ferramenta nova mantida: overlay `/scan` azul no mapa.
- ✅ **Simulador 4-rodas reativado** fiel ao real (geometria real, skid-steer, LiDAR).
- ✅ **unstuck_supervisor** validado ("melhorou pra cacete"): ré furando o collision + escape
  pra frente quando atrás está bloqueado + recovery contextual (parede mapeada → ré aos 2s; novo → 5s).
- ✅ **IMU MPU6050** validada (~99%), fusão de odom funcionando.
- ✅ **Mapa golden** `maps/sala.*` preservado (backup chmod a-w) — não sobrescrever.
- ✅ **CPU da Pi** controlada (mega_bridge 70→~29%, pose_estimator 75→~62% após fixes).

---

## 4. O QUE FAZER (próximos passos)

### Agora / curto prazo
1. **NAV2 "burro"** — fazer ir reto no corredor + giro 90° no lugar (ver §2). 1 mudança por vez.
2. **Ler os CSVs de diagnóstico** na Pi: scan-lag (boneco atrasado) e tensão do BMS no próximo desarme.
3. **Remover logs DBG** (`DBG recov:`) após validar o recovery contextual em campo.
4. **Limpar #2 porta** do SLAM.

### Estratégia sim (médio prazo)
5. Rodar o sim **com `--pi`** e/ou alinhar `nav2_params.yaml` ↔ `nav2_params_pi.yaml`.
6. **Injetar no sim os erros reais dos sensores** (ruído IMU, patinagem de yaw, satura/zona-morta
   do giro, lixo do flow) pra que a confiança bata com o real.
7. Definir **loop de trabalho:** iterar no sim → soltar o real só pra validar em janelas curtas.

### Pendências de validação de campo (quando o robô estiver ligado)
- **Reproduzir o congelamento perto do goal e ler o `freeze_capture.csv`** (deployado `4f8b306`,
  bateria cortou antes). É o teste que decide a causa raiz do "robô burro".
- Validar travessia da door com o mapa novo `sala` (bateria morreu antes — `a39e4b7` deployado).
- Validar ré do recovery contextual (ré aos 2s em parede mapeada).
- Teste de **odometria linear** (ficou pra depois).

---

## 5. Como trabalhar (preferências fixas do dono)

- **Pergunte antes de agir:** dizer o que entendi + o que pretendo + **perguntar**. Confirmar a
  **causa** antes da **solução**.
- **Não pensar demais** (time-box ~30s): resposta curta, ação rápida, 1 coisa por vez.
- **Mudança grande** (reescrita/arquitetura) **não vai blind pro campo**: revisar a fundo + backup
  do último estado bom + anunciar. Default = 1 mudança **pequena** por vez.
- **Em teste no robô: EU leio os logs e diagnostico, o dono só liga/roda/desliga.** Nada de
  instrumentação que obrigue ele a ler/relatar — gravar **CSV/arquivo na Pi** que eu puxo via ssh.
- **Energia:** avisar explicitamente quando o robô precisa ficar **ligado** (captura/validar/reflash)
  vs quando pode **desligar** (codar/compilar/testar offline). Default = desligado.
- **Avisar e esperar o "pode"** antes de abrir janela de captura hands-on.
- Commits **sem** rodapé de co-autoria.

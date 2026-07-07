# 8ª Auditoria de Código — 2026-07-07

> Escopo: tudo que mudou desde a 7ª auditoria (06-11) — path_follower, unstuck
> reescrito, motion_guard, pipeline 2-mux, freeze_capture, configs nav2/collision
> — mais uma re-passada nos nós estáveis (mega_bridge, pose_estimator, fused_odom,
> scan_sanitizer, cmd_vel_to_wheels, sim_actuator_model, utils) e launch/YAMLs.
> Foto no momento da auditoria: `64fc971`, **245 testes passando**.
>
> ⚠️ Regra da casa: aplicar em sessão separada, **1 item por vez**, validar no sim
> antes da Pi. NADA aqui é urgente a ponto de mexer no BASELINE de 07-06 sem teste.

---

## A — Achados com ação recomendada

### A1 🟠 motion_guard virou o SEGUNDO ponto único de falha da autonomia
**Onde:** pipeline `twist_mux_auto → auto_vel_pre → motion_guard → auto_vel_raw →
collision → auto_vel` (`nav2.launch.py:196`, `motion_guard.py:288`).
**O problema:** o failsafe do guard (pass-through) cobre TF/scan indisponível e
`enabled=false` — mas NÃO cobre o **processo morto**. Se o nó crashar (exceção no
timer, OOM), `auto_vel_raw` deixa de ser publicado e a autonomia inteira morre
(humano/unstuck seguem OK). O collision como SPOF foi decisão consciente e
documentada ("collision OBRIGATÓRIO"); o guard entrou DEPOIS na mesma artéria e
herdou o status de SPOF **sem decisão explícita**. Crash fatal em nó nosso já
aconteceu (AttributeError no unstuck, 06-28) — não é hipótese teórica.
**Recomendação (escolher 1):**
- (a) `respawn=True` + `respawn_delay` no Node do launch (barato, cobre crash);
- (b) aceitar e documentar no comentário do launch como o collision;
- (c) os dois.
**Custo:** 1 linha no launch. **Risco:** nenhum.

### A2 🟡 unstuck: `_near_r`/`_near_deg` nunca resetam com scan 100% inválido
**Onde:** `unstuck_supervisor.py:1314-1319` — `if finite.any():` sem `else`.
**O problema:** um scan fresco mas todo inf/0 (glitch do LD06, "abnormal state"
do 1º start) deixa `self._near_r` com o valor **antigo** para sempre. O gate de
segurança do giro (`nearest >= spin_clear`) e a direção (`nearest_deg`) passam a
decidir com dado obsoleto — o giro varre a quina confiando numa folga que pode
não existir mais. Probabilidade baixa, mas é exatamente a classe de bug (dado
congelado) que já causou o giro-fantasma da MEGA.
**Fix:** `else: self._near_r = 0.0` (conservador: sem retorno válido = não gira),
espelhando o que o `_tick` já faz com scan stale.
**Custo:** 2 linhas + 1 teste.

### A3 🟡 `nav2_params.yaml` (dev) é um FÓSSIL perigoso como default
**Onde:** `nav2.launch.py:22` (`default_params = nav2_params.yaml`).
**O problema:** o arquivo dev parou no tempo (~06-08): **NavFn** (vs Theta*),
DWB **sem RotationShim**, `max_vel_theta 0.8` (vs 6.0), costmaps no **/scan cru**,
**sem observation_persistence**, inflação 0.25/0.25, critics antigos, collision no
`scan` cru. O `launch.sh --sim` redireciona pro `_pi` (fidelidade sim=real OK),
mas qualquer `ros2 launch robot_nav nav2.launch.py` direto — ou script futuro que
esqueça o argumento — sobe **silenciosamente** a stack de um mês atrás, e o
sintoma ("robô não gira", "zigue-zague") mandaria a investigação pro lugar errado.
**Recomendação:** trocar o default do launch pra `nav2_params_pi.yaml` e renomear
o velho pra `nav2_params_legacy.yaml` (ou deletar — o git guarda).
**Custo:** 1 linha + rename. **Risco:** zero (ninguém usa o default hoje).

### A4 🟡 motion_guard snapshotta scans BORRADOS durante o giro
**Onde:** `motion_guard.py:135` — o `self._snaps.append(...)` acontece ANTES do
gate `if abs(wz) > c.wz_gate: return` (linha 139).
**O problema:** girando, o TF atrasa e a nuvem projetada sai borrada (medido
06-30: tf_fallback 100%, p99 222 ms). O gate corretamente NÃO avalia durante o
giro — mas o snapshot borrado **entra no deque** e vira a referência "old" dos
primeiros ~0.5 s pós-giro: células/polar erradas → falso móvel logo depois de
girar. É uma fonte residual da família de falsos positivos que vocês caçaram um
a um em 07-03 (persist_frames mitiga, não elimina).
**Fix:** mover o gate pra ANTES do append (não snapshotta girando). Custo
colateral: após giro longo, o snapshot "old" fica mais velho que lookback — o
raycast já tolera (get default inf → só valida com feixe registrado).
**Custo:** mover 4 linhas + ajustar 1 teste.

### A5 🟢 CSVs de campo: flush POR LINHA a 20 Hz castiga o SD da Pi
**Onde:** `path_follower.py:355` (`self._csv_f.flush()` a cada tick com goal) e
`motion_guard.py:424` (flush a cada `auto_vel_pre`, ~20 Hz).
**O problema:** runs de 26 min = ~30k flushes síncronos por arquivo. Não é bug —
mas é desgaste de SD e I/O evitáveis, e o freeze_capture já faz o certo (timer de
flush a cada 2 s, `freeze_capture.py:108`).
**Fix:** replicar o padrão do freeze_capture (flush em timer) nos dois nós.
**Custo:** ~6 linhas por nó. Perda máxima em power-cut: 2 s de log.

### A6 🟢 unstuck: `pinch_fire` age em ~2 s mesmo com "aperto" DESCONHECIDO
**Onde:** `unstuck_supervisor.py:620-621`.
**Observação (não é bug, é assimetria de filosofia):** `near_fire` exige parede
MAPEADA; `pinch_fire` (side_clear < 0.40 por 2 s) dispara sem consultar o mapa.
Pessoa PARADA colada do lado (que o motion_guard nunca latchou, pois não se
moveu) = manobra em 2 s, não os 15 s cautelosos de "desconhecido". Na prática o
dano é limitado (a manobra é gap-gated nos 3 eixos; com algo a <0.40 o giro nem
libera), e o motivo do pinch rápido é legítimo (06-28). **Decisão pro dono:**
aceitar como está (recomendo, campo validou) ou exigir `near_mapped` também no
pinch. Registrado pra ninguém redescobrir esse trade no futuro.

### A7 🟢 Custo do `_tick` do unstuck dá pra cortar ~70% quando ocioso
**Onde:** `unstuck_supervisor.py:1373-1385`.
**O problema:** a cada tick de 10 Hz, MESMO com o robô andando feliz, rodam:
`map_occupied` do near_mapped (raio 0.6 m = ~625 células em Python puro) +
`clearest_heading_offset` (~31 candidatos × front_min_gap vetorizado). São as
duas contas mais caras do nó e o resultado só é consumido quando o robô está
efetivamente parado há ≥2 s.
**Fix (se a CPU da Pi apertar de novo):** computar ambos só quando
`now - anchor_t > stuck_timeout_mapped - 0.5` (pré-aquecimento de 0.5 s antes do
primeiro gate que os usa). Comportamento idêntico, tick ocioso ~3× mais leve.
**Custo:** ~10 linhas. Só vale se a telemetria de CPU justificar — hoje não urge.

---

## B — Higiene / notas (sem ação obrigatória)

- **B1** Untracked na raiz há dias: `worlds/bolsao*.sdf`, `worlds/sala_real.sdf`,
  `bin/map2world.py`, `Laudo_MPU6500.pdf`, `PROVA_MPU6500_NAO_9250.md`,
  `PS-MPU-9250A-01-v1.1.pdf`. Os worlds/tool são úteis (A/B do bolsão 07-06
  dependeu deles) → commitar; os PDFs de laudo, decidir se entram no repo ou
  ficam em `docs/`.
- **B2** `mega_bridge._on_leds` empacota FT_LEDS que o firmware ignora (anel
  comentado). No-op documentado — OK, só lembrar de religar os DOIS lados juntos.
- **B3** `/imu/data` sai no frame BRUTO do sensor; a correção da montagem de
  ponta-cabeça vive só no `imu_yaw_sign` (yaw). Se um dia um EKF/consumidor novo
  usar accel/gyro x-y, vai precisar da rotação completa. Nota pra futuro.
- **B4** `door_crossing` (741 linhas) + máscara de batente do scan_sanitizer
  seguem no repo com o door DESATIVADO no launch. Decisão consciente (06-26);
  reavaliar remoção quando a porta nativa acumular mais campo.
- **B5** freeze_capture assina `scan` com loop Python por feixe a ~10 Hz
  (`_on_scan`) — único loop não-vetorizado da família; irrelevante hoje.
- **B6** `nav2_params.yaml` × `_pi`: além do A3, lembrar que o dev também collide
  com `scan` cru. Se o A3 for aplicado (rename), esse ponto morre junto.

## ✅ O que está BOM (pra constar)

- 245 testes verdes em 2,2 s; padrão "lógica pura + nó cola de I/O" aplicado em
  todos os nós novos — o motion_guard nasceu já testável.
- Os comentários de código com data+motivo+medição são EXEMPLARES — metade desta
  auditoria foi só confirmar que o código faz o que o comentário promete (faz).
- Correções da 7ª auditoria seguem no lugar (EventsExecutor via `utils.spin_node`,
  wheel batching, monotonic freshness).
- Cadeia de segurança coerente de ponta a ponta: gap-gate em TODAS as manobras do
  unstuck, guard nunca escala wz, collision limit por eixo, MEGA com 3 watchdogs.

## Status de aplicação (2026-07-07, mesma data — autorizado pelo dono)

1. ✅ **A2** `42981e4` — near_r/near_deg resetam com scan 100% inválido.
2. ✅ **A1** `e69c50f` — motion_guard com `respawn=True, respawn_delay=1.0`.
3. ✅ **A3** `3b8967b` — default do launch = `nav2_params_pi.yaml`; fóssil
   renomeado `nav2_params_legacy.yaml` com header de aviso.
4. ✅ **A4** `3b1ba9b` — gate de giro ANTES do snapshot + teste de regressão
   (pessoa que chega durante o giro agora é detectada ao terminar). **⏳ validar
   no sim com a "pessoa" teleop** (`bin/teleop-pernas`) antes de ir pra Pi.
5. ✅ **A5** `2d70bf6` — flush em timer 2 s no path_follower e motion_guard.
6. ⏸️ **A6** aceito como está (recomendação da auditoria); **A7** só se a CPU pedir.

Verificação: **246 testes verdes** (245+1), `colcon build` OK, smoke test de 5 s
nos 3 nós alterados (motion_guard, path_follower, unstuck) — todos vivos.
B1 (untracked) segue pendente de decisão do dono.

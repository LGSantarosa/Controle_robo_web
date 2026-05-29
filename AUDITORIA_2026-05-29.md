# Auditoria do projeto Controle_robo_web — 2026-05-29

Quinta passada. Esta sessão tem **dois objetivos**:

1. **Verificar as auditorias anteriores** (completude + coerência com o código de hoje).
2. **Nova auditoria** do código como um todo, com foco onde a 05-27 mandou
   olhar (`trekking_runner`, `pose_estimator`, `nav_metrics`) + o que mudou
   depois dela (anel WS2812 abandonado, fixes de EMI do PMW3901).

> ## ✅ Status de execução (aplicado em 2026-05-29, mesma sessão)
> Por decisão do usuário ("deixa o anel comentado, o resto pode começar a trabalhar"):
> - **A1** — anel WS2812 **comentado** (não removido): `leds.cpp` fora do build via `build_src_filter`, dep FastLED comentada, `#include`/objeto `ring`/case `FT_LEDS` comentados em `main.cpp`. Firmware compila (Flash 5,5 %). Produtores ROS (`/leds/color`, `_led_tick`) **mantidos** pra revival fácil.
> - **A2** — flow: `txFlow()` publica amostra **nula** (q=0) em vez de suprimir a rejeitada por EMI (corrige o viés de ~2× sem acumular lixo — variante mais correta que a proposta original (a)). Nota cross-component no `pose_estimator`.
> - **M1/M2/M3/M4/B2** — limpeza de `platformio.ini`, docstring `mega_bridge.py`, comentário `leds.h`, ~8 pontos do README, mapa de pinos.
> - **05-27 M3/M4/M6** (via meu M5) — Nav2 internos em `log`; clone raso + hash do LiDAR; paralelismo por RAM no `setup_pi.sh`. **M8 pulado** (caduco).
> - **Verificado:** `pio run` ✅, `colcon build --packages-select robot_nav` ✅, `bash -n setup_pi.sh` ✅, `py_compile` ✅.
> - **Pendente de hardware:** reflash da MEGA (`pio run -t upload`) pra A1/A2 valerem no robô; sincronizar git da Pi antes (`project_pi_git_desync.md`).

Auditorias anteriores:
- [AUDITORIA_2026-05-14.md](./AUDITORIA_2026-05-14.md) — 1ª limpeza (D1 collision_monitor removido)
- [AUDITORIA_2026-05-18.md](./AUDITORIA_2026-05-18.md) — 2ª passada (~95 % aplicada)
- [AUDITORIA_2026-05-26.md](./AUDITORIA_2026-05-26.md) — 3ª pós-headless (100 % aplicada)
- [AUDITORIA_2026-05-27.md](./AUDITORIA_2026-05-27.md) — 4ª (parcialmente aplicada via `5fa5d85`)

> Severidades: 🔴 crítica (bloqueia uso real / quebra dados / segurança) — 🟠 alta (bug funcional, comportamento incorreto) — 🟡 média (qualidade/robustez/UX) — 🟢 baixa (cosmético / refator).
>
> **Estado git:** branch `main`, HEAD `1dacaa0` (`tests pwm data`). Sem alterações não-commitadas. **A Pi (192.168.18.95) está dessincronizada** — ver memória `project_pi_git_desync.md` antes de mexer no robô.
>
> **Memórias relevantes pra próxima sessão:**
> - Anel WS2812 **abandonado** (commits `fa769f8`/`8451dea`), trocado por fita fixa de 3 LEDs na 12 V só pra iluminar o PMW3901 — ver `project_led_ring_din_short_2026-05-27.md`.
> - PMW3901 voltou a ler (ID 0x49); lixo ±32768 era EMI do motor no SPI — ver `project_pmw3901_emi_motor.md`.
> - Flow é entrada de fusão "ruidoso-mas-coerente": não precisa ser perfeito, mas 0/lixo saturado não serve (`feedback_flow_fusion_grade.md`).
> - Sem `Co-Authored-By Claude` em commits deste repo.
> - Repo vive em `~/Workspace/Controle_robo_web` (não `~/Controle_robo_web`).
> - Auditoria ≠ execução: registrar decisões, **não** aplicar fixes nesta sessão.

---

# PARTE 1 — Verificação das auditorias anteriores

## Estão completas e bem-feitas?

**Sim.** As quatro são self-contained, organizadas por severidade com `arquivo:linha`,
trazem estado git + memórias + decisões pendentes + checklist. O formato é
consistente com o fluxo preferido (auditar → consolidar num `.md` → executar
em outra sessão). Taxa de aplicação confirmada por leitura do código atual:

| Auditoria | Aplicação confirmada no código de hoje |
|-----------|----------------------------------------|
| 05-18 | C2 (lock trekking) ✅, C3 (RX queue no mega_bridge) ✅, C4 (cmds alinhados) ✅, A5 (try/except `_sanitize_wp`) ✅, A6/M19 (debounce + `button_stable=None`) ✅, A7 (recovery `Flow::read`) ✅, M1 (integração midpoint odom) ✅, M2 (normaliza quaternion) ✅, M17 (guard NaN `cmd_vel_to_wheels`) ✅ |
| 05-26 | C2 (`ClassicBondedOnly` no `_bluez_fixes.sh`) ✅, A6 (idempotência LED) ✅ *(mas ver coerência abaixo)*, twist_mux/web_vel ✅ |
| 05-27 | A1/A2/M5/M9/M10/B2/B5 aplicados via `5fa5d85` ✅ |

A previsão da 05-27 ("próxima auditoria: trekking/pose/nav_metrics") foi
acertada — auditei os três a fundo nesta passada (resultado: **sólidos**, ver Parte 2).

## Problema de coerência: o anel WS2812 foi abandonado DEPOIS da 05-27

Este é o achado central da Parte 1. As auditorias **05-26 e 05-27 gastaram
itens (e o commit `5fa5d85` gastou esforço de fix) em código de LED que ficou
morto poucos dias depois**:

- A 05-26 **A6** ("idempotência do `Ring::transition_`") foi aplicada (`6281a1d`)
  e está em `leds.cpp:43-45`. A 05-26 **M2** / 05-27 **M1**/**B6** corrigiram
  comentários de animação. A 05-27 **M8** propôs um tópico `/leds/state` pra
  disparar o gating do PMW3901.
- Em `fa769f8` e `8451dea` (**posteriores** ao commit da 05-27, `c107c37`) o anel
  foi **abandonado**: `main.cpp` **não chama mais `ring.begin()` nem `ring.tick()`**.
  O gating do flow foi removido (`txFlow()` em `main.cpp:229-230` documenta isso).

Resultado: **todos os itens de LED das 05-26/05-27 são hoje moot** (mexem em
código não-executado), e a 05-27 **M8** não faz mais sentido (não há gating a
disparar). Isso não é culpa das auditorias — elas refletiam o código da data —
mas a próxima execução **não deve aplicar** M8 nem reabrir A6/M1/M2/B6 de LED.

→ A consequência funcional disso (robô perdeu feedback visual; código/dep mortos;
README/comentários stale) virou os achados **A1, M1–M4, B1–B2** da Parte 2.

---

# PARTE 2 — Nova auditoria (código atual)

## 🔴 CRÍTICOS

Nenhum bloqueador funcional. A stack de movimento, odometria, fusão e métricas
está coerente. Os achados abaixo são "comportamento incorreto/morto" e robustez.

---

## 🟠 ALTOS — bugs funcionais / comportamento incorreto

### A1 — Driver do anel WS2812 é código morto, mas ainda é compilado, alimentado por tópico e some sem aviso

- `firmware/mega_bridge/src/leds.cpp` (215 linhas) + `include/leds.h` (68 linhas) — classe `Ring` inteira
- `firmware/mega_bridge/src/main.cpp:41` — `leds::Ring ring;` (objeto global)
- `firmware/mega_bridge/src/main.cpp:67-86` — `FT_LEDS` ainda chama `ring.triggerWaypoint()/setState()/setManual()/clearManual()`
- `firmware/mega_bridge/src/main.cpp:242-272` — `setup()`/`loop()` **NÃO** chamam `ring.begin()` nem `ring.tick()`
- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:198,255-261` — `/leds/color` → `FT_LEDS len=4` → `ring.setManual()` (vai pro vazio)
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py:153,519-545` — `_led_tick/_flash_led/_set_led` publicam cor por modo/chegada

Confirmado por grep: `ring.begin`, `ring.tick`, `ring.setActive`, `ring.setError`,
`ring.gated()` **não aparecem em lugar nenhum** do firmware. O `FastLED.addLeds`
(em `Ring::begin`) e o `FastLED.show` (em `Ring::tick`) nunca executam. Logo:

1. **Funcional:** o robô **perdeu o feedback visual de estado**. O
   `trekking_runner` ainda dedupa e publica `ColorRGBA` (verde=RECORD, ciano=PLAY,
   laranja=chegada) a cada mudança — tudo cai no `setManual()` de um objeto que
   ninguém renderiza. O usuário não vê mais "gravou waypoint" / "chegou".
2. **Recursos:** `leds.cpp` + dep `fastled/FastLED@^3.7.0` continuam linkados
   no ATmega2560 (flash/RAM escassos numa 2560). FastLED não é pequeno.
3. **Confusão:** ~283 linhas de firmware + um caminho de tópico ROS inteiro
   (`/leds/color`) que parecem ativos mas não fazem nada.

**Decisão necessária (D2026-05-29-A):** o robô vai ter feedback visual de novo?
- **(a)** Se **não** (a fita de 3 LEDs é só iluminação fixa na 12 V): remover
  `leds.cpp`/`leds.h`, o `#include "leds.h"`, o objeto `ring`, o case `FT_LEDS`
  em `main.cpp`, a dep FastLED do `platformio.ini`, o sub `/leds/color` no
  `mega_bridge.py`, e o `_led_tick`/`_flash_led`/`_set_led` do `trekking_runner`
  (trocar por `last_msg` que a UI já mostra). Limpa ~350 linhas e libera flash.
- **(b)** Se **sim**, mas via outro hardware: redirecionar o feedback pro
  `PIN_LED` de marco (`io_signals.cpp:14`, pino 8) — um LED simples liga/pisca
  por estado, sem FastLED. Aí o `/leds/color` vira `/light/marker` (Bool) e o
  runner pisca o marco em vez de RGB.
- **(c)** Reviver o anel: improvável (memória diz que o caminho de dados
  MEGA→anel está suspeito de pino queimado). Não recomendado.

> A escolha (a) é a mais provável dado o estado atual. Confirmar com o usuário.

---

### A2 — Flow subestima velocidade quando uma amostra é descartada (firmware) ou perdida (transporte)

- `firmware/mega_bridge/src/sensors_flow.cpp:18-54` — `read()` faz `readMotionCount()` a **cada** chamada (100 Hz)
- `firmware/mega_bridge/src/main.cpp:222-240` — `txFlow()` só publica `FT_FLOW` se `read()` retornar `true`
- `ros2_packages/robot_nav/robot_nav/pose_estimator.py:178-204` — `_on_flow` calcula `v = (dx·m_per_count)/dt`, com `dt = wall-time entre mensagens ROS recebidas`

O PMW3901 acumula deslocamento entre leituras; `readMotionCount()` **zera o
acumulador a cada leitura** (comportamento padrão do sensor — confirmar no fork
`lib/Bitcraze_PMW3901/`). O firmware lê a 100 Hz **sempre**, mas:

- Se a amostra é rejeitada pelo filtro de EMI (`sensors_flow.cpp:44-48`, retorna
  `false`), os counts daquela janela de ~10 ms **foram consumidos/zerados e não
  são publicados** → perdidos.
- A próxima amostra boa carrega só os counts desde a última **leitura** (~10 ms),
  mas o `pose_estimator` mede `dt` como o tempo de parede entre **mensagens
  recebidas** (~20 ms+ porque uma foi pulada).

Consequência: `v_flow = counts_de_10ms / dt_de_20ms` ≈ **metade** da velocidade
real. O viés é proporcional à fração de amostras descartadas. E **a rejeição
acontece justamente manobrando** (`sensors_flow.cpp:37-43` documenta: rodas em
sentidos opostos → EMI), que é quando o flow mais importa na fusão. O mesmo vale
para drops de transporte (flow é `best_effort` a 100 Hz sobre 230400 baud
compartilhado com IMU+STATE — `mega_bridge.py:181`).

Não derruba nada (flow é entrada de fusão, e o `α` cai quando o SQUAL piora),
mas **enviesa a fusão pra baixo exatamente nas manobras** — contradiz o objetivo
"ruidoso-mas-coerente" (`feedback_flow_fusion_grade.md`).

**Fix (escolher um):**
- **(a) Recomendado:** o firmware acumula os counts das amostras rejeitadas e
  soma na próxima amostra publicada (não perde deslocamento); o `dt` do ROS
  passa a casar com a janela real.
  ```cpp
  // em Flow::read(), ao rejeitar: pend_dx_ += dx; pend_dy_ += dy; return false;
  // ao aceitar: dx_ = dx + pend_dx_; dy_ = dy + pend_dy_; pend_dx_=pend_dy_=0;
  ```
  (Cuidado com saturação int16 — clampar a soma.)
- **(b)** O firmware carimba cada `FT_FLOW` com o nº de janelas de 10 ms cobertas
  (1 byte), e o `pose_estimator` usa `n·0.010 s` como `dt` em vez do wall-clock.
- **(c)** Usar `msg.header.stamp` (já preenchido no `mega_bridge.py:414`) em vez
  do `get_clock().now()` de chegada no `_on_flow` — corrige o jitter de chegada
  mas **não** o problema de counts perdidos. Insuficiente sozinho.

---

## 🟡 MÉDIOS — qualidade, robustez, UX

### M1 — `platformio.ini` e o cabeçalho do `main.cpp` ainda anunciam o anel WS2812

- `firmware/mega_bridge/platformio.ini:2` — comentário "anel WS2812"
- `firmware/mega_bridge/platformio.ini` `lib_deps` — `fastled/FastLED@^3.7.0` (só usado pelo `leds.cpp` morto)
- `firmware/mega_bridge/src/main.cpp:9` — `// pino 6 → DIN do anel WS2812 (ANEL DESATIVADO ...)`

Depende da decisão **D2026-05-29-A**. Se (a): remover a dep FastLED daqui
(economiza build + flash) e os comentários. Hoje quem lê o `.ini` acha que o
anel é parte ativa do firmware.

### M2 — Docstring do `mega_bridge.py` lista "anel WS2812" como hardware agregado

- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:6-10` — "Agora a MEGA agrega: ... anel WS2812, relé da luz, LED de marco, botão"
- `mega_bridge.py:25` — doc do tópico `/leds/color` consumido

Stale. Atualizar pra refletir que o anel saiu (e o `/leds/color` é no-op até a
decisão A1).

### M3 — `leds.h` ainda carrega o comentário "TEMP DIAG 2026-05-27" (pino 6→5)

- `firmware/mega_bridge/include/leds.h:6-9` — "movido de 6 → 5 pra testar se pino 6 da MEGA esta' queimado ... Reverter pra 6 se a troca de pino nao resolver."

Esse diagnóstico ficou órfão: o anel foi abandonado, então não há mais "reverter
pra 6". Confunde quem investigar pinos. Remover junto com a decisão A1, ou se o
arquivo sobreviver, trocar por "anel abandonado — pino livre".

### M4 — `README.md` descreve em ~8 pontos um feedback de LED que hoje não acontece

- `README.md:543` — "Aperte ● Gravar ... **O anel de LED fica verde piscando.**"
- `README.md:557` — "...pisca o anel WS2812 e avança pro próximo."
- `README.md:866-867` — tabela de saídas: anel WS2812 `/leds/color`, e relé "pode ser removido — o anel já cobre mudar de cor"
- `README.md:1224` — "anel WS2812 fica vermelho se o BNO055 não responder no I²C"
- `README.md:173, 390, 951, 1031` — diagrama de pinos / leds.h / tópico

Todas essas frases descrevem comportamento não-funcional desde `fa769f8`.
Atualizar em bloco quando a decisão A1 for tomada (mesma PR).

### M5 — Itens da 05-27 ainda ABERTOS (não aplicados em `5fa5d85`) continuam válidos

Reconfirmados contra o código de hoje — **não re-derivar, só executar**:
- **05-27 M3** — `nav2.launch.py:53-108` os 8 nós Nav2 com `output='screen'` poluem o terminal. Ainda assim.
- **05-27 M4** — `setup_pi.sh` clona `ldlidar_stl_ros2` sem pin de versão. Ainda assim.
- **05-27 M6** — `setup_pi.sh` paralelismo hardcoded `-j2` (Pi 5 subutilizada). Ainda assim.
- **05-27 M8** — ❌ **NÃO aplicar**: virou moot com o fim do anel/gating.
- Decisões pendentes **D2026-05-27-A/B** continuam de pé; **D2026-05-27-C** caduca.

---

## 🟢 BAIXOS — cosméticos / refator / confirmações

### B1 — Confirmações de robustez (sem ação) nos nós auditados a fundo nesta passada

Os três nós que a 05-27 marcou como "caixa-preta" foram lidos por inteiro:

- **`pose_estimator.py`** — A3 (saúde do flow) da 05-18 está **resolvido**:
  `/trekking/health` (JSON), warns throttled de `flow stale` e `alpha baixo >2 s`
  (linhas 279-343). Lock em todo estado compartilhado. Integração no mundo com
  `cos(yaw)/sin(yaw)`. Único ponto a melhorar: o `dt` do flow (ver A2).
- **`trekking_runner.py`** — C2 (lock), A5 (try/except sanitize), A6/M19
  (debounce + `button_stable=None`), C4 (set de cmds: `reset/record/save_point/
  play/stop/load_waypoints/clear`) **todos resolvidos**. Máquina de estado limpa.
  Caveat: `mode/waypoints/current_idx` são mutados por callbacks (`_on_cmd`,
  `_control_tick`) **sem** o `_state_lock` (que só cobre pose/cones) — OK
  enquanto for `SingleThreadedExecutor`, frágil se migrar (mesmo caveat já
  documentado no construtor).
- **`nav_metrics.py`** — sólido. Itera `status_list` inteira (não só `[-1]`),
  limpa `_recovery_ids` no goal novo (B16), distância por pose (não integra
  velocidade), `fsync` no CSV, CSV por dia via `end_ts` do clock ROS. B18 da
  05-18 (lambda `n=name`) confirmado correto. Sem achados.

### B2 — `main.cpp:9-11` mapa de pinos lista anel (pino 6) que diverge do `leds.h` (pino 5)

Cosmético e duplamente moot (anel morto). `main.cpp:9` diz pino 6; `leds.h:9`
diz pino 5. Resolver junto da decisão A1.

### B3 — Modelo `m_per_count` do `pose_estimator` é aproximado (counts ≠ pixels)

- `pose_estimator.py:63-97` — `m_per_count = flow_height · tan(fov/pixels)`

Trata 1 count como 1 pixel, o que não é exato pro PMW3901. Funciona porque a
calibração real vive nos sinais/swap (`flow_swap_xy=True`, `flow_x_sign=-1` em
`trekking.launch.py`, calibrados em campo 2026-05-29) e o flow é entrada de
fusão. Sem ação — anotar que a escala absoluta é "calibrada empiricamente, não
derivada da ótica".

---

## Decisões pendentes (consultar usuário antes de aplicar)

- **D2026-05-29-A** — **(bloqueia A1, M1–M4, B2)** O robô terá feedback visual?
  (a) remover todo o caminho de LED + FastLED; (b) redirecionar pro LED de marco
  (pino 8); (c) reviver o anel. Recomendo **(a)** salvo se o usuário quiser
  sinalização de waypoint de volta — aí **(b)**.
- **D2026-05-29-B** — A2 (flow dt): aplicar (a) acúmulo no firmware (mais
  correto, mexe no firmware → reflash) ou (b) byte de nº de janelas (idem) ou
  só (c) `header.stamp` (barato, parcial). Recomendo (a)+(c).
- Herdadas: **D2026-05-27-A** (outputs Nav2), **D2026-05-27-B** (paralelismo Pi 5).

---

## Checklist sugerido (ordem de execução)

### Etapa 0 — Decidir
0. **D2026-05-29-A** (feedback visual) — destrava a maior parte da limpeza.

### Etapa 1 — Altos
1. **A1** — aplicar a opção escolhida em D-A (remover ou redirecionar LED).
2. **A2** — corrigir o `dt`/counts do flow (firmware + `pose_estimator`). **Requer reflash da MEGA** (`pio run -t upload`).

### Etapa 2 — Médios (em bloco com A1)
3. **M1/M2/M3/M4** — limpar `platformio.ini`, docstring `mega_bridge.py`, comentário `leds.h`, e os ~8 pontos do README. Tudo na mesma PR de A1.
4. **M5** — aplicar os itens abertos da 05-27 (M3/M4/M6); **pular M8**.

### Etapa 3 — Baixos
- B2 (pinos no `main.cpp`) junto de A1. B1/B3 = anotação, sem fix.

---

## Notas finais

- **Sem 🔴.** A stack funcional (movimento, odometria, fusão IMU+flow+rodas,
  métricas Nav2, web em modo monitor) está coerente e as 4 auditorias anteriores
  foram majoritariamente aplicadas.
- O tema desta passada é **dívida do anel abandonado**: ~350 linhas de
  firmware/ROS + dep FastLED + ~8 pontos de README + comentários descrevendo um
  feedback visual que sumiu em `fa769f8`. Um único ciclo de decisão (D-A) +
  limpeza resolve A1/M1–M4/B2 de uma vez.
- O único achado técnico **novo** de comportamento é **A2** (viés de velocidade
  do flow em manobra) — sutil, introduzido pelo filtro de EMI da `baca4f3`, e
  relevante porque ataca o flow justamente quando ele importa.
- **Antes de testar no robô:** sincronizar o git da Pi (`project_pi_git_desync.md`).
  A2 e A1(b) exigem reflash da MEGA.
</content>
</invoke>

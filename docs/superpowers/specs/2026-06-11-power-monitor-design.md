# Monitor de tensão das placas hoverboard (power_monitor)

**Data:** 2026-06-11
**Status:** aprovado pelo usuário (design verbal nesta sessão)

## Problema

O BMS de um dos hoverboards desarma "do nada" durante testes de navegação
(39V→6-7V, robô para; botão de emergência + religar volta 39V). Duas hipóteses
vivas, indistinguíveis sem dados:

1. **Stall / rotor bloqueado** — robô encalhado (obstáculo abaixo do plano do
   LiDAR, <21cm, invisível ao collision monitor) com o nav empurrando →
   corrente de rotor bloqueado → sag de tensão → BMS corta por
   sobrecarga/subtensão. Assinatura: setpoint≠0 + RPM≈0 + **tensão afundando
   gradualmente** antes do corte.
2. **Mau contato** (conector/solda na potência) — vibração da manobra abre o
   circuito. Assinatura: **queda instantânea** de tensão sem carga relevante.

Hipótese descartada (2026-06-11): "degrau de comando sem rampa" — a ré do
unstuck parte do repouso (±100 units) e o teleop PS4 comanda mais forte
(6.0 rad/s > 3.0–4.2 do unstuck) sem nunca desarmar.

## Objetivo

Capturar a assinatura elétrica do desarme: tensão por placa correlacionada
com comando vs velocidade medida das rodas, contínua durante a sessão, com
visualização ao vivo na UI web.

## Arquitetura (aprovada: opção A)

Serviço `power_monitor.py` dentro do `controle_web`, no molde do
`nav_metrics.py` (nó rclpy próprio + executor em thread + CSV em `logs/`) e
do `trekking_service.py` (push pro browser via `sock.emit`). Sem colcon
build; roda sempre que a UI roda — exatamente quando há teste de campo.

Rejeitada: nó separado no `robot_nav` (mais peças, colcon, assinatura
duplicada pra UI, sem ganho).

### Entradas (tópicos já existentes, publicados pelo mega_bridge)

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/battery/front`, `/battery/rear` | `sensor_msgs/BatteryState` | V por placa; **0.0 = placa stale** (mega_bridge zera quando a placa não responde >200ms) |
| `/wheel_vel_setpoints` | `robot_interfaces/WheelSpeeds` (conferir import do mega_bridge) | comando enviado à MEGA (units −1000..1000) |
| `/hoverboard/wheel_velocities` | `std_msgs/Float64MultiArray` | medido, ordem FL FR RL RR |

Callbacks apenas guardam o último valor (CPU mínima — a Pi é justa).

### Componentes

1. **`PowerEventDetector`** — classe pura (sem rclpy), testável com pytest.
   Recebe snapshots (t, v_front, v_rear, setpoints[4], measured[4]) e devolve
   eventos:
   - `SAG`: queda >3.0V em <1.0s numa placa (vs. média móvel curta).
   - `TRIP`: tensão <30.0V, OU placa que reportava >20V passa a reportar 0
     (stale = BMS cortou a alimentação da placa).
   - `STALL`: |setpoint|>50 units com |medido|≈0 (<tolerância) por >0.5s em
     qualquer roda.
   Limiares como parâmetros do construtor (default acima).
2. **`PowerMonitor`** — serviço: nó rclpy (subs acima) + timer 10 Hz que tira
   snapshot, passa no detector, escreve CSV bufferizado (flush ≤1s) e a 2 Hz
   emite `power_update` pro browser. WARN no logger a cada evento.
3. **CSV** — `controle_web/logs/power/power_YYYY-MM-DD_HHMMSS.csv` (um por
   sessão): `ts, v_front, v_rear, set_fl, set_fr, set_rl, set_rr, meas_fl,
   meas_fr, meas_rl, meas_rr, stall, event`.
4. **UI** — chip de telemetria no painel: `F 39.2V · R 39.1V`. Verde normal;
   amarelo com SAG/STALL ativo; vermelho piscando com TRIP/stale. Evento
   `power_update`: `{v_front, v_rear, front_ok, rear_ok, stall, event}`.

### Integração

- `app.py`: instanciar como o `NavMetricsCollector` (try/except, não derruba
  o app se falhar; shutdown junto). Passa `socketio`.
- `static/js/client.js`: listener `power_update` atualiza o chip.
- `templates/index.html` (+ css se preciso): markup do chip.

### Erros / bordas

- Tópicos mudos (MEGA desligada): chip mostra "—" cinza; sem spam de WARN.
- BatteryState usa `voltage` em V (mega_bridge divide raw/100).
- O monitor NÃO publica nada em tópico ROS — só observa (zero risco ao robô).

### Testes

`controle_web/test_power_monitor.py` (pytest, sem ROS): detector — sag
gradual → SAG antes de TRIP; queda instantânea sem stall → TRIP sem SAG/STALL
prévio; stall sem queda → só STALL; escrita/rotação de CSV; histerese (evento
não repete a cada tick enquanto a condição persiste).

## Critério de sucesso

Na próxima sessão de campo com desarme, o CSV responde sozinho: havia STALL e
sag gradual antes do TRIP (→ sobrecarga/BMS) ou a tensão caiu de 39V pra 0/6V
num tick sem carga (→ mau contato).

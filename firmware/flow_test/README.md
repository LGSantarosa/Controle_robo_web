# Teste do PMW3901 (optical flow) — `flow_test`

Sketch isolado pra validar o sensor de fluxo óptico **PMW3901** na Arduino MEGA,
sem o resto do robô no caminho (sem hoverboard, sem IMU, sem ROS, sem o protocolo
do `mega_bridge`). Serve pra responder "o sensor está lendo?" de forma definitiva e
pra diagnosticar fiação/SPI.

Escrito durante a sessão de depuração de **2026-05-28**.

---

## O que o sketch faz

`src/main.cpp` lê o registrador de **Product ID (0x00)** do PMW3901 a ~5 Hz e imprime
o byte cru no monitor serial (115200). Quando o ID vem certo (`0x49`), ele inicia o
sensor e passa a imprimir `dx`/`dy` (motion count) — o mesmo `readMotionCount()` que o
firmware real usa.

Usa o **fork local** do driver (`../mega_bridge/lib/Bitcraze_PMW3901`, 125 kHz / SPI_MODE0)
via `lib_extra_dirs`, então o timing de SPI é idêntico ao do `mega_bridge`.

### Legenda do byte de ID (pra diagnóstico de fiação)

| Leitura | Significado |
|---|---|
| `ID=0x49` | sensor respondendo (OK) |
| `ID=0xFF` | MISO "solto pra cima" — sem energia no sensor / shifter mudo / MISO desconectado |
| `ID=0x00` | MISO preso em nível baixo — fio no GND / curto / furo errado |
| varia/oscila | mau contato intermitente |

---

## Como rodar (MEGA conectada na Pi como `/dev/ttyACM0`)

> A porta tem que estar **livre** — se o `mega_bridge`/teleop estiver rodando, ele
> segura o `/dev/ttyACM0` e o upload falha (`failed to leave programming mode`).
> Pare a stack antes (`tmux kill-session -t robo` ou Ctrl+C no launch).

```bash
cd firmware/flow_test
pio run -t upload                          # compila + grava (~10 s com a porta livre)

# Ler o serial em texto:
stty -F /dev/ttyACM0 115200 raw -echo
timeout 10 cat /dev/ttyACM0
```

Pelo firmware real (`mega_bridge`), o mesmo dado sai no tópico ROS — lembre do QoS
**best_effort** (senão o echo não recebe nada):

```bash
ros2 topic echo /optical_flow --qos-reliability best_effort   # x=dx, y=dy, z=quality
```

---

## Fiação (MEGA 2560 ↔ PMW3901, via conversor de nível 5V↔3.3V)

Ver também `memory/project_mega_pinout.md`.

| Sinal | Pino MEGA | Observação |
|---|---|---|
| MISO | **50** | reto (não cruzado): MISO do sensor → 50 |
| MOSI | **51** | MOSI do sensor → 51 |
| SCK  | **52** | |
| CS   | **10** | **NÃO é o pino 53 (SS)** — o 53 fica vazio |
| 3V3  | — | alimentação do sensor (3,3 V), pelo lado LV do shifter |
| GND  | — | comum |

O sensor fica atrás de um **level shifter MOSFET (tipo I²C)**. Ele precisa dos dois
rails ligados ao mesmo tempo: **LV = 3,3 V** e **HV = 5 V**.

### Erros de fiação encontrados (e corrigidos) em 2026-05-28

1. **2 fios de força sem conexão** chegando 0 V no sensor → `begin()` falhava (`0xFF`).
2. **CS/CLK em pino errado.** O usuário se guiava por "tem 3 V no pino?" pra identificar
   — e isso **engana**: o SCK (52) é clock, fica ~0 V parado (correto); o MISO (50) fica
   ~3 V por causa do pull-up do shifter. Logo "3 V" marca o MISO, não "o pino certo".
   Estado certo: 50=MISO, 51=MOSI, 52=SCK, 53=vazio, **CS=10**.

Com isso o ID virou `0x49` na hora.

---

## Resultados

### Sensor + chão: FUNCIONA (via flow_test + empurrão na mão)

Empurrando o robô **com a mão** sobre o chão real (textura normal, ~12 cm de altura),
o sensor rastreia limpo — `dy` proporcional ao movimento, `dx` ≈ 0:

```
ID=0x49  dx=0  dy=13
ID=0x49  dx=0  dy=28
ID=0x49  dx=1  dy=50
ID=0x49  dx=-1 dy=54
ID=0x49  dx=2  dy=56
ID=0x49  dx=1  dy=58   <- pico do empurrão
ID=0x49  dx=0  dy=31
ID=0x49  dx=1  dy=15   <- desacelerando
...
```

Conclusão: **sensor, SPI, firmware do sketch, alimentação, luz e chão estão OK.**
Calibração observada: **frente/trás = eixo `dy`** (`dx` fica ~0).

### Sob o `mega_bridge` dirigindo: EM ABERTO

No **mesmo chão**, com o firmware completo (`mega_bridge`) e dirigindo pelos **motores**,
o `/optical_flow` dá **`0.0`** (a maior parte) ou **lixo saturado `±16384`/`±32768`** (só
no eixo `y`). Valor real esperado a 0,3 m/s seria da ordem de **dezenas** de counts — então
±32768 é *phantom motion*/leitura corrompida, não fluxo real.

Isso **não** é o piso (o sensor rastreia bem nele via flow_test) nem a luz (forte, confirmado).
Sobram duas causas, **ainda não isoladas**:
1. **Carga de SPI** do firmware completo (Serial1/2 a 115200 + USB 230400 + I²C, concorrentes
   com o flow a 100 Hz, através do shifter MOSFET marginal) corrompendo as leituras.
2. **Vibração dos motores** durante a direção.

---

## Próximos passos

1. **Isolar carga-de-SPI vs vibração:** rodar o `mega_bridge` (carga cheia) e **empurrar o
   robô na mão** (motores parados). Se o `/optical_flow` ficar limpo → era vibração. Se der
   lixo → é a carga de SPI (aí: espaçar/validar as leituras, reduzir concorrência, ou bit-bang).
2. **Direção x/y:** confirmar/configurar o mapeamento dos eixos do flow em relação ao robô
   (observado: frente/trás = `dy`; falta o lateral e os sinais).
3. **Implementar o FIXME C5** (`mega_bridge/src/sensors_flow.cpp`): ler o **SQUAL** real
   (registro `0x07`), hoje fixo em `quality = 0`. Sem isso o `pose_estimator` zera o peso do
   flow (alpha≈0) e o sensor não entra na fusão de jeito nenhum. Com o SQUAL real, a fusão
   usa o flow quando a leitura é boa e o ignora quando não — que é o papel dele (uma entrada
   a mais junto de odometria das rodas + IMU + LiDAR).

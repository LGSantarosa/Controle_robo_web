# Bancada: placa de hover nova + MEGA no PC — 2026-09-01

**Objetivo:** descobrir se uma roda de hoverboard gira, usando uma placa de hover
nova (nunca usada) e uma Arduino MEGA ligada direto no PC, sem Pi e sem ROS.

**Resultado:** ela gira. As duas rodas giraram nos dois sentidos. A placa está boa,
o cabo está bom, o protocolo do projeto funciona. O que travava eram **duas
travas de segurança do firmware da própria placa**, descritas na seção
[Como fazer funcionar](#como-fazer-funcionar).

---

## 1. Montagem

| item | valor |
|---|---|
| MEGA | `/dev/ttyACM0` — VID:PID `2a03:0042`, serial `55632313039351D05132` |
| placa de hover | 1 placa, alimentada por bateria própria (~36 V, pack 10S) |
| rodas | começou com 1 roda; **terminou com 2** (foi obrigatório, ver §5) |
| link MEGA↔placa | Serial1, 115200 baud |

Fiação (idêntica ao `CONEXOES.txt`, canal chamado de "FRENTE" no projeto):

```
   MEGA                          PLACA DE HOVER
   pino 18 (TX1) ──── verde ───→ RX     (a MEGA fala)
   pino 19 (RX1) ←─── azul  ───── TX    (a placa responde)
   GND           ──── preto ───── GND   (obrigatório)
                       (VCC, 4º fio, fica solto)
```

Serial é cruzado: TX de um lado sempre no RX do outro.

### Diferenças deste PC em relação à Pi

- **Não existe `/dev/mega`** — o `setup_udev.sh` nunca rodou nesta máquina.
- O `platformio.ini` do `mega_bridge` tem `upload_port = /dev/mega` fixo, então
  **todo upload precisa de `--upload-port /dev/ttyACM0`**.
- O `~/.platformio/packages/` não tinha `tool-avrdude`: nenhum upload jamais
  havia saído deste PC. O pio baixou na primeira gravação.

---

## 2. Protocolo (do próprio projeto)

`firmware/mega_bridge/include/hoverboard.h` — comando de 8 bytes, feedback de 18:

```c
Command  { uint16_t start=0xABCD; int16_t steer; int16_t speed; uint16_t checksum; }
Feedback { uint16_t start; int16_t cmd1, cmd2, speedR, speedL,
           batVoltage, boardTemp; uint16_t cmdLed, checksum; }
checksum = XOR de todos os campos anteriores
```

Enviado a 50 Hz. A placa tem watchdog: parou de receber, ela zera os motores.

**`steer`/`speed` não escolhem lado.** A placa distribui internamente:
`speedR = speed − steer`, `speedL = speed + steer`. Com `steer=0`, os dois canais
recebem o mesmo valor. Um único par de fios serial comanda os dois motores —
**não existe "mandar pro outro lado", e jumpear as saídas de motor entre si
queima a placa.**

---

## 3. Cronologia dos testes

### 3.1 A MEGA estava muda (não era a placa)

`test_mega.py` não recebeu nenhum frame STATE. Sonda em 230400/115200/57600/9600
baud: **0 bytes em todos**. A MEGA não tinha o firmware do projeto — era outra
placa, com sketch de fábrica. Gravado o `mega_bridge` (assinatura `0x1e9801`,
ATmega2560, 14828 bytes verificados) e a MEGA passou a publicar 50 Hz.

### 3.2 Falso positivo: eco por curto 18↔19

Com a primeira placa (uma que o dono nunca tinha usado), a sonda recebia
**exatamente os bytes que mandava** — `CD AB 00 00 00 00 CD AB`, o próprio frame
em little-endian — **800 bytes recebidos para 800 enviados, idênticos nos 5 bauds
testados**. Eco byte-a-byte independente de baud é curto/loopback entre TX e RX,
nunca resposta de dispositivo (baud errado embaralharia).

Teste de continuidade em firmware (18 como GPIO, 19 como entrada) deu 78% HIGH
flutuando, não curto duro: **contato ruim, fio meio solto**. Trocada a placa por
uma igual à do projeto, o problema sumiu.

**Lição:** se o feedback "responde" com o mesmo conteúdo que você mandou, é fio,
não é placa.

### 3.3 Comunicação fechada

Com a placa boa e a fiação correta: `batF = 36,45 V`, 99 frames STATE em 2 s,
400 frames válidos contra 1 checksum ruim (0,25%, lixo do boot). **O critério do
projeto pra "placa viva" é `batF > 1,0 V`.**

### 3.4 A placa aceitava e não acionava

A sonda expôs `cmd1`/`cmd2`, que o `mega_bridge` **não repassa** (o frame STATE
dele só leva RPM e bateria):

```
[GO] enviei speed=300 | cmd1=0 cmd2=300  spdR=0 spdL=0  bat=36.10V temp=29.4
```

`cmd2` = 300 exato, `cmd1` = 0 (o steer que mandamos). **A placa recebeu, validou
o checksum e aceitou.** E mesmo assim: bateria cravada, temperatura sem variar uma
décima, rotação 0.

### 3.5 Hall: bom nos dois canais

Sonda com `speed=0` fixo, girando a roda **com a mão** (a placa lê os halls
continuamente, mesmo desacionada):

| canal | pico lido |
|---|---|
| esquerdo | −94 a **+121** |
| direito | −67 a **+83** |

Sinal invertendo com o sentido = os 3 sensores bons, decodificação correta, roda
mecanicamente livre. E **diz em qual canal a roda está** — resolveu a confusão dos
dois soquetes sem chute.

### 3.6 Velocidades testadas

| comando | duração | resultado |
|---|---|---|
| `speed=1` | 1 s | nada — é 0,1% de ±1000, não vence o atrito |
| `speed=±100` | 1 s | nada, **corrente zero** |
| `speed=300` | 0,1 s | nada quando desarmada; **girou** quando armada |
| `speed=300` | 0,5 s | **girou as duas**, nos dois sentidos |
| rampa 30/60/100 | 1 s cada | **girou as duas** |

Ou seja: **não era velocidade nem duração.** O mesmo comando ora funcionava, ora
não — o que mudava era o estado de armação da placa.

---

## 4. Como ler o estado da placa (tabela de diagnóstico)

Estes quatro sinais resolvem praticamente qualquer caso:

| sinal | o que significa |
|---|---|
| `batF = 0,00 V` | a placa não responde: cabo, GND, ou placa desligada |
| feedback ecoa o que você mandou | curto TX↔RX, a placa não está na conversa |
| `cmd2` acompanha o `speed` | a placa **recebeu e aceitou** o comando |
| `cmd2` fica em 0 | a placa ignora serial (firmware sem `CONTROL_SERIAL_USART2`) |
| **bateria e temperatura IMÓVEIS** | **corrente zero** — não energizou |
| bateria afunda / temperatura sobe | corrente circulou: fase conectada, ordem errada ou travado |
| `spdR`/`spdL` mexem girando na mão | hall bom, e indica **em qual canal** a roda está |
| **beep constante** | **`errCode` ou timeout ativo — saída travada** |
| **beep muda** | **armou** |

O beep é o único indicador do `errCode`: **nenhum campo do feedback o expõe.**

Cuidado com a interpretação de "corrente zero": ela é compatível com fase aberta
**e também** com estágio desabilitado. Foi o erro central desta sessão — passamos
vários testes caçando fase aberta e MOSFET queimado quando a placa apenas não
tinha armado. O que derrubou a hipótese foi uma observação física: *"assim que eu
ligo, a roda fica dura"* — frenagem magnética só existe com o circuito fechado,
o que prova fases conectadas e enrolamento íntegro.

---

## 5. Como fazer funcionar

O `hoverboard-firmware-hack-FOC` tem duas travas empilhadas. As duas precisam cair.

### Trava 1 — precisa das DUAS rodas

O `enable` só arma com `!errCode_esquerdo && !errCode_direito`. Com **uma roda
só**, o canal vazio lê hall inválido, marca erro, e **os dois canais ficam
mortos** — inclusive o que tem roda boa.

Por isso testar cada canal separadamente não levava a lugar nenhum: o bloqueio é
global, não do canal. E por isso o robô do projeto nunca esbarrou nisso — lá cada
placa sempre teve duas rodas.

### Trava 2 — precisa girar as rodas COM A MÃO

O `errCode` de hall **não limpa com a roda parada**. O controlador precisa ver uma
sequência real de transições dos 3 sensores; parado, ele lê um estado estático e
mantém o erro.

### Receita (sem intervalo entre os passos)

1. As **duas** rodas plugadas — fases e hall de cada uma **no mesmo canal**.
2. Placa ligada, bateria própria.
3. MEGA mandando `speed=0` a 50 Hz, **continuamente**.
4. **Girar as duas rodas com a mão**, ~15 s, enquanto a MEGA fala.
5. **O beep muda** — esse é o sinal de que armou.
6. Comandar. Funciona na hora.

Se a placa ficar um tempo sem receber comando, ela cai em timeout e **volta a
travar**; zeros mandados depois não recuperam. Tem que refazer o passo 4.

---

## 6. Armadilhas

- **`pio run -t upload` ACIONA as rodas.** O `setup()` do sketch roda no boot, e a
  gravação reseta a MEGA. Aconteceu duas vezes nesta sessão sem intenção. **No robô
  real isso o faria sair andando ao flashear.** A sonda chegou a ganhar um gatilho
  (espera um byte pela USB antes de acionar) — reponha o gatilho antes de gravar
  qualquer coisa que acione motor.
- **Não jumpeie as saídas de motor entre si.** Cada canal é uma ponte inversora
  independente; ligá-las curto-circuita meio-braços de MOSFET.
- **Fases e hall da mesma roda têm que ir no mesmo canal.** Separados, um canal
  fica sem comutação e o outro sem corrente — e o sintoma é idêntico a "placa
  queimada".
- **Frame corrompido pode passar no checksum** (1 em 65 mil). Um deles envenenou
  uma linha de base e imprimiu "queda de −250 V". A sonda ganhou faixa de sanidade
  (descarta `batVoltage` fora de 10–60 V).

---

## 7. Ferramenta

`firmware/hover_probe/` — sonda de bancada, **não commitada**. Mostra `cmd1`,
`cmd2`, `boardTemp` e os halls, que o `mega_bridge` não repassa e que o
`test_mega.py` não tem como diagnosticar (ele diz "placa não responde" e para).

```bash
cd firmware/hover_probe && pio run -t upload --upload-port /dev/ttyACM0
# ATENÇÃO: gravar já aciona, se o sketch não tiver gatilho

# voltar a MEGA pro sistema do projeto:
cd firmware/mega_bridge && pio run -t upload --upload-port /dev/ttyACM0
```

Diagnóstico sem movimento, com o `mega_bridge` gravado:

```bash
python3 firmware/mega_bridge/tools/test_mega.py --port /dev/ttyACM0 --front-only --speed 0
```

---

## 8. Estado final e pendências

- **MEGA:** regravada com o `mega_bridge`.
- **`firmware/hover_probe/`:** não commitada. Se for commitar, **repor o gatilho
  antes** — não deve entrar no repositório uma versão que arranca ao ser gravada.
- **Não testado:** comportamento com as duas placas (frente + trás) juntas; a
  segunda trava pode aparecer de novo se alguma roda for desconectada em campo.
- **Vale checar no robô real:** se o robô ficar parado tempo suficiente pra placa
  cair em timeout, ele pode precisar da mesma receita de rearmar. Nunca observamos
  isso em campo — mas nunca procuramos.

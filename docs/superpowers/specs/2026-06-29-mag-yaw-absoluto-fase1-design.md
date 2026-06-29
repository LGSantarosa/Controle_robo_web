# Yaw absoluto via magnetômetro — Fase 1 (ler + calibrar + validar)

**Data:** 2026-06-29
**Status:** design aprovado, implementação iniciada
**Contexto:** [[project_imu_mpu9250]] — a IMU agora é MPU9250 (gyro/accel validados, montada plana
no centro). Ela tem um magnetômetro interno (AK8963) que o MPU6050 não tinha.

## Objetivo (do dono)
Yaw com **referência absoluta** pra:
- **(A)** matar a deriva de longo prazo do yaw integrado do gyro;
- **(C)** corrigir o "erro no giro" — o gyro tem ~+2.2% de escala que **acumula** a cada giro.

A bússola (mag) dá a direção absoluta que corrige os dois. A fusão gyro+mag é a Fase 2.

## Escopo desta fase
**Fase 1 = só descobrir se o magnetômetro é USÁVEL neste robô**, principalmente sob a EMI dos
motores. **Não muda nada da navegação.** Se o mag passar → Fase 2 (fusão). Se a EMI detonar →
para aqui, sem custo na pose que está boa.

**Muda:** só o firmware da MEGA (passa a ler e transmitir o mag) + uma ferramenta offline de
análise. **NÃO muda:** `mega_bridge.py`, `pose_estimator.py`, launch, nav (intactos).

## O que vai ser feito

### 1. Firmware — ler o AK8963 e transmitir (`firmware/mega_bridge/`)
- `sensors_imu.*`: na init, habilitar **bypass** (`INT_PIN_CFG 0x37`, bit `BYPASS_EN`) →
  AK8963 acessível em I²C `0x0C`. Conferir WHO_AM_I (`0x00` = 0x48). Ler a **correção de fábrica
  ASA** (fuse ROM `0x10-0x12`) e aplicá-la. Modo contínuo 100 Hz / 16-bit (`CNTL1 = 0x16`).
- Leitura por amostra: `ST1`(data ready) → `HXL..HZH` (int16 LE) → **ler `ST2`** (obrigatório
  pra liberar a próxima amostra; checar overflow `HOFL`). Se HOFL/erro → marca leitura inválida.
- **Alinhar eixos:** o frame do AK8963 é rotacionado vs o do gyro/accel (mag X = sensor Y,
  mag Y = sensor X, mag Z = −sensor Z). Alinhar no firmware pro mesmo frame do gyro.
- Novos getters `mx()/my()/mz()` (µT ou raw escalado) + `magOk()`. Bias do gyro intacto.
- `protocol.h`: novo tipo de frame **`FT_MAG = 0x83`**. `main.cpp`: enviar mx,my,mz (int16,
  milli-unidade, igual ao gyro) num frame 0x83, a ~20-50 Hz. **Frame 0x82 (gyro) inalterado.**

### 2. Ferramenta offline (`firmware/mega_bridge/tools/mag_check.py` ou `controle_web/tools/`)
- Lê `/dev/mega` cru (230400), decodifica os frames `0x83` (igual ao decoder de gyro já usado
  nos testes ad-hoc desta sessão).
- **Modo coleta/calibração:** grava mx,my,mz enquanto o robô gira 360°. Calcula **hard-iron**
  (offset = (max+min)/2 por eixo) e **soft-iron** (escala simples: raio médio / raio do eixo).
  Cospe as constantes (pra usar na Fase 2). Salva o dump em CSV pra eu reanalisar.
- **Modo validação:** aplica a calibração, calcula `heading = atan2(my_cal, mx_cal)` e reporta:
  estabilidade parado, e a variação ponto-a-ponto.

## Critérios de validação (go/no-go da Fase 2)
1. **Estável parado:** heading não vaga mais que ~±2-3° com o robô imóvel.
2. **Consistente com o gyro:** girar o robô 90° → o heading do mag muda ~90° (±alguns graus),
   no mesmo sentido.
3. **🔴 Sobrevive à EMI (decisivo):** repetir (1) e (2) **com os motores LIGADOS** (teleop leve).
   Se o heading pular/girar sozinho com motor → mag NÃO usável → não fazer a Fase 2 (ou só usar
   com gate forte). Se aguentar → segue.

## Fora de escopo (Fase 1)
- Fusão no pose_estimator (Fase 2).
- Compensação de inclinação (tilt) — robô anda em piso plano/nivelado → assume nivelado (YAGNI;
  revisar só se o chão for irregular).
- Mudanças no `mega_bridge.py` (a validação é por leitura crua; o parse do 0x83 no bridge entra
  na Fase 2, junto com a fusão).

## Riscos
- **EMI do motor** (principal incógnita — é justamente o que o critério 3 mede).
- **Hard-iron do próprio robô** (massa metálica/ímãs do hover perto) → a calibração trata o
  offset estático; campo VARIÁVEL com a corrente do motor é o risco real (cai no critério 3).
- I²C: ler o AK8963 via bypass adiciona transações no barramento — monitorar se não atrapalha o
  stream do gyro (a MEGA já tem `Wire.setWireTimeout`+watchdog).

## Como validar (precisa robô LIGADO)
Flashear (`pio run -t upload` pela Pi) → rodar `mag_check.py` no modo coleta girando 360° →
modo validação parado/90°/com-motor. Eu leio os dados e dou o veredito.

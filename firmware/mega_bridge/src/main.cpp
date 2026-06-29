// Firmware da Arduino MEGA 2560 — ponte do PC para o robô 4 rodas.
//
// Fluxo:
//   USB (PC) ↔ Serial       (protocolo agregado, frames 0xAA 0x55, 230400 baud)
//   Serial1 ↔ placa hoverboard FRENTE  (controla FL+FR, SerialCommand 0xABCD)
//   Serial2 ↔ placa hoverboard TRÁS    (controla RL+RR, SerialCommand 0xABCD)
//   I2C ↔ MPU9250 IMU (giro + accel; mag AK8963 p/ yaw absoluto é TODO)
//   SPI ↔ PMW3901 optical flow (CS = pino 10)
//   (anel WS2812 COMENTADO — ver AUDITORIA_2026-05-29 A1. O driver vive em
//    leds.cpp/leds.h mas está FORA do build via build_src_filter no
//    platformio.ini, e os usos abaixo estão comentados. Iluminação do chão
//    pro PMW3901 agora é fita fixa de LEDs na 12 V — não controlada por SW.)
//   pino 7  → relé da luz
//   pino 8  → LED de sinalização do marco
//   pino 9  → botão de partida (pull-up interno)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include <avr/wdt.h>

#include "protocol.h"
#include "hoverboard.h"
#include "sensors_imu.h"
#include "sensors_flow.h"
// #include "leds.h"   // anel WS2812 comentado — ver AUDITORIA_2026-05-29 A1
#include "io_signals.h"

constexpr uint32_t PC_BAUD               = 230400;
constexpr uint32_t HOVER_BAUD            = 115200;
constexpr uint8_t  PMW_CS                = 10;
constexpr uint32_t TX_HOVERBOARD_PERIOD  = 20;     // 50 Hz comandos pras placas
constexpr uint32_t TX_STATE_PERIOD       = 20;     // 50 Hz STATE pro PC
constexpr uint32_t TX_IMU_PERIOD         = 20;     // 50 Hz IMU pro PC
constexpr uint32_t TX_FLOW_PERIOD        = 10;     // 100 Hz FLOW pro PC
constexpr uint32_t SETPOINT_TIMEOUT_MS   = 500;    // watchdog: se PC sumir, zera motores

protocol::Decoder         pc_decoder;
hoverboard::FeedbackParser fb_front;
hoverboard::FeedbackParser fb_rear;

sensors::Imu   imu_dev;
sensors::Flow  flow_dev{PMW_CS};
// leds::Ring     ring;   // anel WS2812 comentado — ver AUDITORIA_2026-05-29 A1

struct Setpoint { int16_t steer; int16_t speed; };
static Setpoint sp_front      = {0, 0};
static Setpoint sp_rear       = {0, 0};
static uint32_t last_setpoint = 0;

static uint32_t last_tx_hover = 0;
static uint32_t last_tx_state = 0;
static uint32_t last_tx_imu   = 0;
static uint32_t last_tx_flow  = 0;

static void handlePcFrame(uint8_t type, uint8_t len, const uint8_t* p) {
    switch (type) {
        case protocol::FT_SET_SPEED: {
            if (len != 8) return;
            int16_t sf, vf, sr, vr;
            memcpy(&sf, p + 0, 2);
            memcpy(&vf, p + 2, 2);
            memcpy(&sr, p + 4, 2);
            memcpy(&vr, p + 6, 2);
            sp_front      = {sf, vf};
            sp_rear       = {sr, vr};
            last_setpoint = millis();
            break;
        }
        // case protocol::FT_LEDS:  — anel WS2812 COMENTADO (AUDITORIA_2026-05-29 A1).
        //   O mega_bridge ainda pode publicar /leds/color → FT_LEDS; o frame é
        //   silenciosamente ignorado aqui (cai no default) até o anel ser
        //   reativado. Pra reativar: descomentar este case + o include/objeto
        //   `ring` acima, o ring.begin()/tick() no setup()/loop(), e religar a
        //   dep FastLED + remover leds.cpp do build_src_filter no platformio.ini.
        //
        //   len=1: id de estado (ver enum leds::State); 5=WAYPOINT, 0xFF=auto.
        //   len=4: RGB + pattern (modo manual).
        //   if (len == 1) {
        //       const uint8_t id = p[0];
        //       if (id == 0xFF)                             ring.clearManual();
        //       else if (id == (uint8_t)leds::State::WAYPOINT) ring.triggerWaypoint();
        //       else                                        ring.setState(static_cast<leds::State>(id));
        //   } else if (len == 4) {
        //       ring.setManual(p[0], p[1], p[2], p[3]);
        //   }
        //   break;
        case protocol::FT_RELAY: {
            if (len != 2) return;
            io_signals::setRelay(p[0] != 0);
            io_signals::setMarkerLed(p[1] != 0);
            break;
        }
        default:
            break;
    }
}

static void pumpPcSerial() {
    while (Serial.available()) {
        if (pc_decoder.feed((uint8_t)Serial.read())) {
            handlePcFrame(pc_decoder.type(), pc_decoder.len(), pc_decoder.payload());
        }
    }
}

static void pumpHoverboardFeedback() {
    while (Serial1.available()) fb_front.feed((uint8_t)Serial1.read());
    while (Serial2.available()) fb_rear.feed((uint8_t)Serial2.read());
}

static void txHoverboard() {
    const uint32_t now = millis();
    if (now - last_tx_hover < TX_HOVERBOARD_PERIOD) return;
    last_tx_hover = now;
    if (now - last_setpoint > SETPOINT_TIMEOUT_MS) {
        sp_front = {0, 0};
        sp_rear  = {0, 0};
    }
    hoverboard::sendCommand(Serial1, sp_front.steer, sp_front.speed);
    hoverboard::sendCommand(Serial2, sp_rear.steer,  sp_rear.speed);
}

static void txState() {
    const uint32_t now = millis();
    if (now - last_tx_state < TX_STATE_PERIOD) return;
    last_tx_state = now;

    // Convenção: placa FRENTE → speedL_meas=FL, speedR_meas=FR
    //            placa TRÁS   → speedL_meas=RL, speedR_meas=RR
    // Se a placa parou de responder (cabo, alimentação), republicar o
    // último frame para sempre poluiria a odometria com RPMs antigos.
    // Threshold de 200 ms: placa responde a ~50 Hz, então 10 frames
    // perdidos já indica problema real.
    const bool front_stale = fb_front.stale(now);
    const bool rear_stale  = fb_rear.stale(now);
    const auto& ff = fb_front.last();
    const auto& fr = fb_rear.last();

    int16_t rpm_FL = front_stale ? 0 : ff.speedL_meas;
    int16_t rpm_FR = front_stale ? 0 : ff.speedR_meas;
    int16_t rpm_RL = rear_stale  ? 0 : fr.speedL_meas;
    int16_t rpm_RR = rear_stale  ? 0 : fr.speedR_meas;
    int16_t batF   = front_stale ? 0 : ff.batVoltage;
    int16_t batR   = rear_stale  ? 0 : fr.batVoltage;
    uint8_t btn    = io_signals::readButton() ? 1 : 0;

    // faultF / faultR: bit 0 = placa stale (sem feedback há > 200 ms).
    // Outros bits reservados para próximos diagnósticos (over-temp, brownout, etc.).
    uint8_t faultF = front_stale ? 0x01 : 0;
    uint8_t faultR = rear_stale  ? 0x01 : 0;
    // sensor_flags: bit 0 = imu_ok, bit 1 = flow_ok. Lido no bridge para
    // publicar /system/health (JSON) sem polling I²C/SPI do lado do PC.
    uint8_t sensor_flags = (uint8_t)((imu_dev.ok() ? 0x01 : 0) |
                                     (flow_dev.ok() ? 0x02 : 0));

    uint8_t buf[16];
    memcpy(buf + 0,  &rpm_FL, 2);
    memcpy(buf + 2,  &rpm_FR, 2);
    memcpy(buf + 4,  &rpm_RL, 2);
    memcpy(buf + 6,  &rpm_RR, 2);
    memcpy(buf + 8,  &batF,   2);
    memcpy(buf + 10, &batR,   2);
    buf[12] = faultF;
    buf[13] = faultR;
    buf[14] = btn;
    buf[15] = sensor_flags;

    protocol::writeFrame(Serial, protocol::FT_STATE, buf, sizeof(buf));
}

static int16_t f_to_milli(double v) {
    if (v >  32.0) v =  32.0;
    if (v < -32.0) v = -32.0;
    return (int16_t)lround(v * 1000.0);
}

static void txImu() {
    const uint32_t now = millis();
    if (now - last_tx_imu < TX_IMU_PERIOD) return;
    last_tx_imu = now;
    if (!imu_dev.ok())   return;
    if (!imu_dev.read()) return;

    // MPU9250: por enquanto só giro (rad/s) + accel (m/s²), frame BRUTO do sensor.
    // Sem quaternion — não há orientação absoluta. O yaw é integrado da taxa do
    // giro no pose_estimator (que também corrige a montagem de ponta-cabeça).
    int16_t gx = f_to_milli(imu_dev.gx());
    int16_t gy = f_to_milli(imu_dev.gy());
    int16_t gz = f_to_milli(imu_dev.gz());
    int16_t ax = f_to_milli(imu_dev.ax());
    int16_t ay = f_to_milli(imu_dev.ay());
    int16_t az = f_to_milli(imu_dev.az());

    uint8_t buf[12];
    memcpy(buf + 0,  &gx, 2);
    memcpy(buf + 2,  &gy, 2);
    memcpy(buf + 4,  &gz, 2);
    memcpy(buf + 6,  &ax, 2);
    memcpy(buf + 8,  &ay, 2);
    memcpy(buf + 10, &az, 2);

    protocol::writeFrame(Serial, protocol::FT_IMU, buf, sizeof(buf));
}

static void txFlow() {
    const uint32_t now = millis();
    if (now - last_tx_flow < TX_FLOW_PERIOD) return;
    last_tx_flow = now;

    const bool valid = flow_dev.read();
    // Sensor caído / em recovery (ok()==false): não publica. O pose_estimator
    // marca o flow como stale via flow_timeout e cai pras rodas.
    if (!flow_dev.ok()) return;

    // AUDITORIA_2026-05-29 A2: amostra rejeitada por EMI do motor (valid==false,
    // mas o sensor está VIVO) é publicada como NULA (dx=dy=0, quality=0) em vez
    // de suprimida. Suprimir abria um buraco na cadência: o PMW3901 zera o
    // acumulador a cada leitura, então a próxima amostra BOA carregava só ~10 ms
    // de counts enquanto o pose_estimator media dt pelo intervalo entre
    // mensagens recebidas (~20 ms) → velocidade subestimada ~2x justo na manobra.
    // Os counts rejeitados são lixo (não dá pra acumulá-los), mas publicar nulo
    // mantém ~100 Hz: a janela ruim entra na fusão com quality=0 (α≈0 → usa
    // rodas) e a amostra boa seguinte fica com dt correto.
    // Anel desativado (iluminação é fita fixa na 12V): sem gating de flash.
    int16_t dx = 0, dy = 0;
    uint8_t q  = 0;
    if (valid) {
        dx = flow_dev.dx();
        dy = flow_dev.dy();
        q  = flow_dev.quality();
    }

    uint8_t buf[5];
    memcpy(buf + 0, &dx, 2);
    memcpy(buf + 2, &dy, 2);
    buf[4] = q;
    protocol::writeFrame(Serial, protocol::FT_FLOW, buf, sizeof(buf));
}

// Guarda anti-reset-loop do watchdog (project_mega_i2c_hang, 2026-06-09).
// Roda em .init3 — logo após o reset, ANTES do setup() — pra DESARMAR o WDT e
// limpar o MCUSR em todo boot. Sem isto, o bootloader stk500v2 da 2560 (que não
// trata o WDT) poderia ser re-resetado por um WDT ainda armado e cair em
// reset-loop ("brick" até power-cycle). Com a guarda, o WDT só fica ativo dentro
// do loop() (re-armado no fim do setup), nunca durante o bootloader nem a
// calibração bloqueante do giro. Padrão canônico do AVR.
void wdt_init(void) __attribute__((naked, used, section(".init3")));
void wdt_init(void) {
    MCUSR = 0;
    wdt_disable();
}

void setup() {
    Serial.begin(PC_BAUD);
    Serial1.begin(HOVER_BAUD);
    Serial2.begin(HOVER_BAUD);

    io_signals::begin();

    Wire.begin();
    Wire.setClock(400000);
    // Anti-hang do I²C — causa raiz do "MEGA para do nada" (project_mega_i2c_hang,
    // 2026-06-09). SEM timeout, um lockup do barramento (EMI do motor desincroniza
    // a máquina de estado TWI) trava o `Wire` PRA SEMPRE dentro de mpu_.getEvent()
    // → o loop morre, para de mandar frames e de comandar o hover (sintoma: LED ON
    // aceso, TX apagado, /dev/ttyACM0 vivo mas mudo). Com timeout de 25 ms + reset
    // do TWI no estouro, getEvent() RETORNA false e a recuperação por software já
    // existente (sensors_imu.cpp: re-init a cada 2 s) finalmente dispara.
    Wire.setWireTimeout(25000 /* us */, true /* reset_with_timeout */);
    imu_dev.begin();

    SPI.begin();
    flow_dev.begin();

    // Sem setpoint ainda — não marcamos last_setpoint pra ficar IDLE assim que sair do BOOT.
    last_setpoint = 0;

    // Arma o watchdog SÓ agora (fim do setup): a calibração do giro acima é
    // bloqueante (~600 ms de I²C) e estouraria um WDT curto. 2 s dá margem
    // confortável sobre o loop normal (<20 ms, ~50–100× folga → zero risco de
    // disparo falso que recalibraria o bias) e é MAIOR que o tempo do bootloader
    // (~1 s) → o WDT nunca dispara durante o bootloader (ver wdt_init/.init3).
    // Se QUALQUER coisa travar o loop >2 s (lockup I²C residual, Serial.write pro
    // USB preso por starvation de CPU na Pi, etc.), a MEGA reseta e volta a
    // streamar sozinha — em vez do hang permanente (ON aceso, TX apagado).
    wdt_enable(WDTO_2S);
}

void loop() {
    // Alimenta o watchdog a cada iteração (loop normal <20 ms ≪ 2 s). Se o loop
    // travar (não chegar aqui), o WDT reseta a MEGA em 2 s. Ver setup()/wdt_init.
    wdt_reset();
    // pumpPcSerial drena o buffer do USB (64 B). Com PC_BAUD=230400 + I²C do
    // MPU6050 bloqueando o loop, em pico chega a >64 B entre ticks — drenar uma
    // vez só perde bytes. Chamadas extras no meio mantêm o buffer com folga.
    pumpPcSerial();
    pumpHoverboardFeedback();
    txHoverboard();
    pumpPcSerial();
    txState();
    txImu();
    pumpPcSerial();
    txFlow();
}

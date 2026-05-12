// Firmware da Arduino MEGA 2560 — ponte do PC para o robô 4 rodas.
//
// Fluxo:
//   USB (PC) ↔ Serial       (protocolo agregado, frames 0xAA 0x55, 230400 baud)
//   Serial1 ↔ placa hoverboard FRENTE  (controla FL+FR, SerialCommand 0xABCD)
//   Serial2 ↔ placa hoverboard TRÁS    (controla RL+RR, SerialCommand 0xABCD)
//   I2C ↔ BNO055 IMU
//   SPI ↔ PMW3901 optical flow (CS = pino 10)
//   pino 6  → DIN do anel WS2812
//   pino 7  → relé da luz
//   pino 8  → LED de sinalização do marco
//   pino 9  → botão de partida (pull-up interno)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <string.h>

#include "protocol.h"
#include "hoverboard.h"
#include "sensors_imu.h"
#include "sensors_flow.h"
#include "leds.h"
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
leds::Ring     ring;

struct Setpoint { int16_t steer; int16_t speed; };
static Setpoint sp_front      = {0, 0};
static Setpoint sp_rear       = {0, 0};
static uint32_t last_setpoint = 0;
static bool     imu_ok        = false;

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
        case protocol::FT_LEDS: {
            // len=1: id de estado (ver enum leds::State).
            //        5 (WAYPOINT) usa triggerWaypoint pra ter auto-timeout de 3 s.
            //        0xFF sai do override manual e volta ao automático.
            // len=4: RGB + pattern (modo manual, compat com ROS2 antigo).
            if (len == 1) {
                const uint8_t id = p[0];
                if (id == 0xFF) {
                    ring.clearManual();
                } else if (id == (uint8_t)leds::State::WAYPOINT) {
                    ring.triggerWaypoint();
                } else {
                    ring.setState(static_cast<leds::State>(id));
                }
            } else if (len == 4) {
                ring.setManual(p[0], p[1], p[2], p[3]);
            }
            break;
        }
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
    const auto& ff = fb_front.last();
    const auto& fr = fb_rear.last();

    int16_t rpm_FL = ff.speedL_meas;
    int16_t rpm_FR = ff.speedR_meas;
    int16_t rpm_RL = fr.speedL_meas;
    int16_t rpm_RR = fr.speedR_meas;
    int16_t batF   = ff.batVoltage;
    int16_t batR   = fr.batVoltage;
    uint8_t btn    = io_signals::readButton() ? 1 : 0;

    uint8_t buf[16];
    memcpy(buf + 0,  &rpm_FL, 2);
    memcpy(buf + 2,  &rpm_FR, 2);
    memcpy(buf + 4,  &rpm_RL, 2);
    memcpy(buf + 6,  &rpm_RR, 2);
    memcpy(buf + 8,  &batF,   2);
    memcpy(buf + 10, &batR,   2);
    buf[12] = 0;
    buf[13] = 0;
    buf[14] = btn;
    buf[15] = 0;

    protocol::writeFrame(Serial, protocol::FT_STATE, buf, sizeof(buf));
}

static int16_t f_to_q14(double v) {
    if (v >  1.999) v =  1.999;
    if (v < -2.000) v = -2.000;
    return (int16_t)lround(v * 16384.0);
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

    const auto& q = imu_dev.quat();
    const auto& g = imu_dev.gyro();
    const auto& a = imu_dev.accel();

    int16_t qw = f_to_q14(q.w());
    int16_t qx = f_to_q14(q.x());
    int16_t qy = f_to_q14(q.y());
    int16_t qz = f_to_q14(q.z());
    int16_t gx = f_to_milli(g.x());
    int16_t gy = f_to_milli(g.y());
    int16_t gz = f_to_milli(g.z());
    int16_t ax = f_to_milli(a.x());
    int16_t ay = f_to_milli(a.y());
    int16_t az = f_to_milli(a.z());

    uint8_t buf[20];
    memcpy(buf + 0,  &qw, 2);
    memcpy(buf + 2,  &qx, 2);
    memcpy(buf + 4,  &qy, 2);
    memcpy(buf + 6,  &qz, 2);
    memcpy(buf + 8,  &gx, 2);
    memcpy(buf + 10, &gy, 2);
    memcpy(buf + 12, &gz, 2);
    memcpy(buf + 14, &ax, 2);
    memcpy(buf + 16, &ay, 2);
    memcpy(buf + 18, &az, 2);

    protocol::writeFrame(Serial, protocol::FT_IMU, buf, sizeof(buf));
}

static void txFlow() {
    const uint32_t now = millis();
    if (now - last_tx_flow < TX_FLOW_PERIOD) return;
    last_tx_flow = now;
    if (!flow_dev.ok()) return;
    flow_dev.read();

    int16_t dx = flow_dev.dx();
    int16_t dy = flow_dev.dy();
    uint8_t buf[5];
    memcpy(buf + 0, &dx, 2);
    memcpy(buf + 2, &dy, 2);
    buf[4] = flow_dev.quality();

    protocol::writeFrame(Serial, protocol::FT_FLOW, buf, sizeof(buf));
}

void setup() {
    Serial.begin(PC_BAUD);
    Serial1.begin(HOVER_BAUD);
    Serial2.begin(HOVER_BAUD);

    io_signals::begin();

    ring.begin();   // entra em BOOT (pulso branco curto)

    Wire.begin();
    Wire.setClock(400000);
    imu_ok = imu_dev.begin();

    SPI.begin();
    flow_dev.begin();

    // Sem setpoint ainda — não marcamos last_setpoint pra ficar IDLE assim que sair do BOOT.
    last_setpoint = 0;
}

void loop() {
    pumpPcSerial();
    pumpHoverboardFeedback();
    txHoverboard();
    txState();
    txImu();
    txFlow();

    const bool active = (last_setpoint != 0) &&
                        (millis() - last_setpoint < SETPOINT_TIMEOUT_MS);
    ring.setActive(active);
    ring.setError(!imu_ok);
    ring.tick();
}

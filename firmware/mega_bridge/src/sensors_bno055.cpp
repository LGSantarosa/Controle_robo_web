#include "sensors_bno055.h"

namespace sensors {

// --- registradores BNO055 (página 0; é a única que usamos) ------------------
namespace {
constexpr uint8_t REG_CHIP_ID     = 0x00;   // = 0xA0
constexpr uint8_t REG_PAGE_ID     = 0x07;
constexpr uint8_t REG_ACC_DATA    = 0x08;   // ACC(6) MAG(6) GYR(6) contíguos → 1 burst
constexpr uint8_t REG_QUA_DATA    = 0x20;   // w,x,y,z (8 bytes, LITTLE-endian)
constexpr uint8_t REG_CALIB_STAT  = 0x35;   // sys<<6 | gyr<<4 | acc<<2 | mag
constexpr uint8_t REG_UNIT_SEL    = 0x3B;
constexpr uint8_t REG_OPR_MODE    = 0x3D;
constexpr uint8_t REG_PWR_MODE    = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;

constexpr uint8_t CHIP_ID         = 0xA0;
constexpr uint8_t MODE_CONFIG     = 0x00;
constexpr uint8_t MODE_NDOF       = 0x0C;   // fusão 9 eixos + heading absoluto
constexpr uint8_t PWR_NORMAL      = 0x00;

// UNIT_SEL: bit1=1 → gyro em rad/s (senão °/s); bit0=0 → accel em m/s².
// Os demais bits ficam 0 (euler em °, temp em °C, orientação "Windows").
// Pedimos rad/s ao CHIP em vez de converter aqui: menos conta por amostra e
// menos chance de divergir da escala documentada.
constexpr uint8_t UNIT_SEL_ROS    = 0x02;

// Escalas de saída (datasheet 3.6.4):
constexpr float LSB_PER_RPS       = 900.0f;    // gyro, com UNIT_SEL bit1=1
constexpr float LSB_PER_MS2       = 100.0f;    // accel
constexpr float LSB_PER_UT        = 16.0f;     // mag
constexpr float LSB_PER_QUAT      = 16384.0f;  // quaternion (2^14), fixo

// Boot da BNO055 depois do power-on/reset (datasheet: POR ~650 ms). O begin()
// é chamado DEPOIS do MPU no setup() justamente pra a calibração de bias do
// giro (~600 ms, bloqueante) contar como parte dessa espera; o restante é
// coberto aqui e, se ainda assim o chip não responder, o recovery de 2 s do
// read() tenta de novo — sem travar o loop.
constexpr uint16_t BOOT_DELAY_MS  = 700;
}  // namespace

bool Bno055::writeReg_(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Bno055::readRegs_(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    // repeated-start, igual ao driver do MPU: segura o barramento pro read.
    if (Wire.endTransmission(false) != 0) return false;
    const uint8_t got = Wire.requestFrom(addr_, n);
    if (got != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

bool Bno055::tryInit_(uint8_t addr) {
    addr_ = addr;
    uint8_t id = 0;
    if (!readRegs_(REG_CHIP_ID, &id, 1)) return false;
    if (id != CHIP_ID) {
        if (!first_boot_) return false;
        // No BOOT o chip ainda pode estar acordando (responde 0x00 nos
        // primeiros ms). Uma segunda chance curta cobre isso. Fora do boot NÃO
        // esperamos: um endereço mudo aqui é sensor ausente, e dormir 50 ms a
        // cada tentativa de recovery roubaria tempo do loop de controle.
        delay(50);
        if (!readRegs_(REG_CHIP_ID, &id, 1) || id != CHIP_ID) return false;
    }

    // Toda configuração precisa acontecer em CONFIGMODE.
    if (!writeReg_(REG_OPR_MODE, MODE_CONFIG)) return false;
    delay(25);                                  // op→config: 19 ms (datasheet 3.3.1)

    // RST_SYS (reset por software) SÓ no boot. Ele custa um boot inteiro da
    // BNO055 (~700 ms BLOQUEANTES) — no boot isso é de graça, com o robô
    // parado; em runtime seria perigoso: 700 ms sem alimentar o loop estoura o
    // SETPOINT_TIMEOUT_MS (500 ms) do hoverboard e o robô daria um solavanco de
    // parada no meio da manobra a cada tentativa de recovery. Em runtime a
    // reconfiguração abaixo (barata, ~50 ms) já religa o chip nos casos que
    // importam (glitch de barramento, cabo reencaixado); um travamento que só o
    // reset resolve fica pro próximo boot.
    if (first_boot_) {
        writeReg_(REG_SYS_TRIGGER, 0x20);       // RST_SYS: parte de um estado conhecido
        delay(BOOT_DELAY_MS);                   // reset = novo boot completo
        // O endereço não muda, mas o chip volta em CONFIGMODE com os defaults —
        // reconfirma o chip-id antes de seguir.
        if (!readRegs_(REG_CHIP_ID, &id, 1) || id != CHIP_ID) return false;
    }

    writeReg_(REG_PWR_MODE, PWR_NORMAL);
    delay(10);
    writeReg_(REG_PAGE_ID, 0x00);               // registradores de dado vivem na página 0
    writeReg_(REG_SYS_TRIGGER, 0x00);           // clock interno (breakouts sem cristal externo)
    delay(10);
    writeReg_(REG_UNIT_SEL, UNIT_SEL_ROS);      // gyro rad/s, accel m/s²
    delay(10);
    if (!writeReg_(REG_OPR_MODE, MODE_NDOF)) return false;
    delay(25);                                  // config→op: 7 ms (datasheet 3.3.1)
    return true;
}

bool Bno055::begin() {
    // 0x28 é o default dos breakouts (ADR/COM3 em GND); 0x29 com ADR em VCC.
    ok_ = tryInit_(0x28) || tryInit_(0x29);
    first_boot_ = false;   // as próximas chamadas são recovery: sem reset, sem espera
    return ok_;
}

bool Bno055::read() {
    // Mesma política do MPU: barramento caído → re-init a cada 2 s, sem
    // bloquear o loop nem derrubar os outros sensores.
    if (!ok_) {
        const uint32_t now = millis();
        if (now - last_recover_ms_ > 2000) {
            last_recover_ms_ = now;
            begin();
        }
        return false;
    }

    // ACC(6) + MAG(6) + GYR(6) são contíguos a partir de 0x08 → um burst só.
    // (Não emendamos o quaternion no mesmo burst: iria a 32 bytes, o limite
    // EXATO do buffer do Wire no AVR — sem folga nenhuma pra erro.)
    uint8_t b[18];
    if (!readRegs_(REG_ACC_DATA, b, sizeof(b))) {
        ok_ = false;
        return false;
    }
    uint8_t q[8];
    if (!readRegs_(REG_QUA_DATA, q, sizeof(q))) {
        ok_ = false;
        return false;
    }
    uint8_t cal = 0;
    if (!readRegs_(REG_CALIB_STAT, &cal, 1)) {
        ok_ = false;
        return false;
    }

    // Todos os dados da BNO055 são LITTLE-endian (ao contrário do MPU).
    const int16_t rax = (int16_t)((uint16_t)b[1]  << 8 | b[0]);
    const int16_t ray = (int16_t)((uint16_t)b[3]  << 8 | b[2]);
    const int16_t raz = (int16_t)((uint16_t)b[5]  << 8 | b[4]);
    const int16_t rmx = (int16_t)((uint16_t)b[7]  << 8 | b[6]);
    const int16_t rmy = (int16_t)((uint16_t)b[9]  << 8 | b[8]);
    const int16_t rmz = (int16_t)((uint16_t)b[11] << 8 | b[10]);
    const int16_t rgx = (int16_t)((uint16_t)b[13] << 8 | b[12]);
    const int16_t rgy = (int16_t)((uint16_t)b[15] << 8 | b[14]);
    const int16_t rgz = (int16_t)((uint16_t)b[17] << 8 | b[16]);

    ax_ = rax / LSB_PER_MS2;
    ay_ = ray / LSB_PER_MS2;
    az_ = raz / LSB_PER_MS2;
    mx_ = rmx / LSB_PER_UT;
    my_ = rmy / LSB_PER_UT;
    mz_ = rmz / LSB_PER_UT;
    gx_ = rgx / LSB_PER_RPS;
    gy_ = rgy / LSB_PER_RPS;
    gz_ = rgz / LSB_PER_RPS;

    qw_ = (int16_t)((uint16_t)q[1] << 8 | q[0]) / LSB_PER_QUAT;
    qx_ = (int16_t)((uint16_t)q[3] << 8 | q[2]) / LSB_PER_QUAT;
    qy_ = (int16_t)((uint16_t)q[5] << 8 | q[4]) / LSB_PER_QUAT;
    qz_ = (int16_t)((uint16_t)q[7] << 8 | q[6]) / LSB_PER_QUAT;

    calib_ = cal;
    return true;
}

}  // namespace sensors

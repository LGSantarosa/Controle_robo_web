// imu_diag — prova se um módulo "MPU9250" é 9250 genuíno (com mag AK8963) ou
// um 6500 sem mag. Mesmo método do laudo PROVA_MPU6500_NAO_9250.md:
//   1) scanner I²C (quem dá ACK no barramento)
//   2) WHO_AM_I (0x75) do chip inercial em 0x68/0x69
//   3) bypass (PWR_MGMT_1=0, USER_CTRL=0, INT_PIN_CFG=0x02) → expõe o AK8963
//   4) scanner de novo + WIA (0x00) do AK8963 em 0x0C (0x48 = mag presente)
//   5) se tem mag: lê ASA (fuse ROM) + uma amostra em µT (prova que está vivo)
// Roda em LOOP a cada ~3 s → dá pra testar as 2 placas trocando no conector
// sem reflashear (cortar o 5V antes de trocar).

#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_USER_CTRL   = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_WHO_AM_I    = 0x75;

constexpr uint8_t AK_ADDR  = 0x0C;
constexpr uint8_t AK_WIA   = 0x00;   // = 0x48 se o mag existe
constexpr uint8_t AK_ST1   = 0x02;
constexpr uint8_t AK_HXL   = 0x03;
constexpr uint8_t AK_CNTL1 = 0x0A;
constexpr uint8_t AK_ASAX  = 0x10;
constexpr uint8_t AK_WHO   = 0x48;
constexpr float   AK_UT_PER_LSB = 0.15f;  // 16-bit: 4912 µT / 32760

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

// Varre o barramento e imprime quem dá ACK; retorna quantos responderam.
uint8_t scanBus(const __FlashStringHelper* rotulo) {
    Serial.print(rotulo);
    Serial.print(F(" enderecos que respondem:"));
    uint8_t achados = 0;
    for (uint8_t a = 1; a < 127; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F(" 0x"));
            Serial.print(a, HEX);
            ++achados;
        }
    }
    if (achados == 0) Serial.print(F(" NENHUM (fiacao? VCC?)"));
    Serial.println();
    return achados;
}

const __FlashStringHelper* nomeDoChip(uint8_t who) {
    switch (who) {
        case 0x68: return F("MPU6050 (sem mag)");
        case 0x70: return F("MPU6500 (SEM mag)");
        case 0x71: return F("MPU9250 (com mag)");
        case 0x73: return F("MPU9255 (com mag)");
        default:   return F("desconhecido");
    }
}

void runDiag() {
    Serial.println(F("=== DIAG MAG imu_diag ==="));

    // 1) scanner antes do bypass
    scanBus(F("ANTES do bypass: "));

    // 2) WHO_AM_I em 0x68 e 0x69 (AD0)
    uint8_t mpu = 0, who = 0;
    for (uint8_t a = 0x68; a <= 0x69 && !mpu; ++a) {
        uint8_t v = 0;
        if (readRegs(a, REG_WHO_AM_I, &v, 1)) { mpu = a; who = v; }
    }
    if (!mpu) {
        Serial.println(F("VEREDITO: NENHUM MPU em 0x68/0x69 — confira SDA/SCL/VCC/GND"));
        Serial.println(F("=== FIM ===\n"));
        return;
    }
    Serial.print(F("MPU em 0x")); Serial.print(mpu, HEX);
    Serial.print(F("  WHO_AM_I(0x75)=0x")); Serial.print(who, HEX);
    Serial.print(F("  -> ")); Serial.println(nomeDoChip(who));

    // 3) bypass pra expor o AK8963 ao host
    writeReg(mpu, REG_PWR_MGMT_1, 0x00);   // acorda
    writeReg(mpu, REG_USER_CTRL, 0x00);    // I2C master interno OFF
    writeReg(mpu, REG_INT_PIN_CFG, 0x02);  // BYPASS_EN
    delay(20);

    // 4) scanner depois + WIA do AK8963
    scanBus(F("DEPOIS do bypass:"));
    uint8_t wia = 0xFF;
    const bool wia_ok = readRegs(AK_ADDR, AK_WIA, &wia, 1);
    Serial.print(F("AK8963 em 0x0C  WIA(0x00)="));
    if (wia_ok) { Serial.print(F("0x")); Serial.print(wia, HEX); }
    else        { Serial.print(F("sem resposta")); }
    Serial.println(F("   -> 0x48 = magnetometro PRESENTE"));

    const bool tem_mag = wia_ok && wia == AK_WHO;

    // 5) mag presente: ASA de fabrica + uma amostra em µT (prova de vida)
    if (tem_mag) {
        writeReg(AK_ADDR, AK_CNTL1, 0x00); delay(10);   // power down
        writeReg(AK_ADDR, AK_CNTL1, 0x0F); delay(10);   // fuse ROM
        uint8_t asa[3] = {128, 128, 128};
        readRegs(AK_ADDR, AK_ASAX, asa, 3);
        Serial.print(F("ASA fabrica = "));
        Serial.print(asa[0]); Serial.print(' ');
        Serial.print(asa[1]); Serial.print(' ');
        Serial.println(asa[2]);
        writeReg(AK_ADDR, AK_CNTL1, 0x00); delay(10);
        writeReg(AK_ADDR, AK_CNTL1, 0x16); delay(100);  // continuo 100 Hz, 16-bit

        uint8_t st1 = 0;
        bool amostra = false;
        for (uint8_t tenta = 0; tenta < 20 && !amostra; ++tenta) {
            if (readRegs(AK_ADDR, AK_ST1, &st1, 1) && (st1 & 0x01)) {
                uint8_t b[7];
                if (readRegs(AK_ADDR, AK_HXL, b, 7) && !(b[6] & 0x08)) {
                    const int16_t hx = (int16_t)((b[1] << 8) | b[0]);
                    const int16_t hy = (int16_t)((b[3] << 8) | b[2]);
                    const int16_t hz = (int16_t)((b[5] << 8) | b[4]);
                    const float fx = hx * ((asa[0] - 128) / 256.0f + 1.0f) * AK_UT_PER_LSB;
                    const float fy = hy * ((asa[1] - 128) / 256.0f + 1.0f) * AK_UT_PER_LSB;
                    const float fz = hz * ((asa[2] - 128) / 256.0f + 1.0f) * AK_UT_PER_LSB;
                    Serial.print(F("amostra mag (uT): "));
                    Serial.print(fx, 1); Serial.print(' ');
                    Serial.print(fy, 1); Serial.print(' ');
                    Serial.println(fz, 1);
                    Serial.println(F("  (campo da Terra ~25-65 uT total; 0 0 0 fixo = suspeito)"));
                    amostra = true;
                }
            }
            delay(20);
        }
        if (!amostra) Serial.println(F("mag respondeu WIA mas NAO entregou amostra (DRDY nunca subiu)"));
    }

    // Veredito consolidado
    Serial.print(F("VEREDITO: "));
    if (who == 0x71 || who == 0x73) {
        if (tem_mag) Serial.println(F("MPU9250/9255 GENUINO — magnetometro PRESENTE ✔"));
        else         Serial.println(F("WHO_AM_I de 9250 mas SEM mag em 0x0C — anomalo (clone?)"));
    } else if (tem_mag) {
        Serial.println(F("mag presente mas WHO_AM_I nao e de 9250 — anomalo (clone?)"));
    } else {
        Serial.print(nomeDoChip(who));
        Serial.println(F(" — SEM magnetometro (nao e um 9250 genuino)"));
    }
    Serial.println(F("=== FIM ===\n"));
}
}  // namespace

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // Mesma protecao do mega_bridge: modulo ruim/EMI nao pode lockar a TWI.
    Wire.setWireTimeout(25000 /*us*/, true /*reset no timeout*/);
    delay(500);
    Serial.println(F("\nimu_diag: teste roda a cada 3 s — pode trocar o modulo (corta o 5V antes)"));
}

void loop() {
    if (Wire.getWireTimeoutFlag()) {   // barramento travou no ciclo anterior
        Wire.clearWireTimeoutFlag();
        Wire.begin();
    }
    runDiag();
    delay(3000);
}

#include <Arduino.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

// ---- Diagnostico SPI do PMW3901 na Arduino MEGA ----
// Le' o registrador de Product ID (0x00) DIRETO, a 5 Hz, e imprime o byte cru.
// Serve pra descobrir POR QUE o begin() falha, sem chutar fiacao:
//
//   ID sempre 0x00  -> MISO sem dado: sensor sem 3V3, MISO solto ou em GND,
//                      ou shifter sem alimentacao no lado de baixo.
//   ID sempre 0xFF  -> MISO "solto pra cima": ninguem dirige a linha — shifter
//                      nao esta' passando ou o sensor esta' mudo/sem energia.
//   ID flutua/varia -> contato intermitente (mexa nos jumpers e veja mudar).
//   ID = 0x49       -> SENSOR FALANDO. Ai o sketch ja' inicia e mostra dx/dy.
//
// SPI: 50(MISO) 51(MOSI) 52(SCK), CS=10, 125 kHz, MODE0 (igual firmware real).
// LED onboard "L" (pino 13) alterna a cada leitura = sketch rodando.

constexpr uint8_t CS_PIN = 10;
constexpr uint8_t HB_PIN = 13;

static const SPISettings PMW_SPI(125000, MSBFIRST, SPI_MODE0);

Bitcraze_PMW3901 flow(CS_PIN);
static bool    inited   = false;
static bool    hb       = false;
static uint32_t last_ms = 0;

// Replica registerRead() do fork: CS low, manda reg (bit7=0=leitura), le' 1 byte.
static uint8_t regRead(uint8_t reg) {
    reg &= ~0x80u;
    SPI.beginTransaction(PMW_SPI);
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(50);
    SPI.transfer(reg);
    delayMicroseconds(50);
    uint8_t v = SPI.transfer(0);
    delayMicroseconds(100);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

static void printHex(uint8_t b) {
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
}

void setup() {
    pinMode(HB_PIN, OUTPUT);
    Serial.begin(115200);

    SPI.begin();
    pinMode(CS_PIN, OUTPUT);
    // reset de CS + power-on-reset, igual ao begin() do driver
    digitalWrite(CS_PIN, HIGH); delay(1);
    digitalWrite(CS_PIN, LOW);  delay(1);
    digitalWrite(CS_PIN, HIGH); delay(1);

    Serial.println(F("=== Diagnostico SPI PMW3901 (CS=10, 125kHz, MODE0) ==="));
    Serial.println(F("Esperado: ID=0x49"));
    Serial.println(F("0x00=sem dado no MISO | 0xFF=MISO solto/shifter mudo | varia=mau contato"));
}

void loop() {
    if (millis() - last_ms < 200) return;   // 5 Hz, legivel
    last_ms = millis();

    hb = !hb;
    digitalWrite(HB_PIN, hb);

    uint8_t id = regRead(0x00);

    if (id != 0x49) {
        inited = false;
        Serial.print(F("ID=0x"));
        printHex(id);
        Serial.println(F("  (sensor NAO respondendo — ver legenda acima)"));
        return;
    }

    // ID certo: garante init uma vez, depois mostra movimento.
    if (!inited) {
        inited = flow.begin();
        Serial.print(F("ID=0x49  SENSOR OK — begin()="));
        Serial.println(inited ? F("OK, lendo dx/dy:") : F("FALHOU (estranho, ID veio certo)"));
        return;
    }

    int16_t dx = 0, dy = 0;
    flow.readMotionCount(&dx, &dy);
    uint8_t squal = flow.readSqual();   // baseline limpo p/ comparar com o mega_bridge
    Serial.print(F("ID=0x49  dx="));
    Serial.print(dx);
    Serial.print(F("  dy="));
    Serial.print(dy);
    Serial.print(F("  squal="));
    Serial.println(squal);
}

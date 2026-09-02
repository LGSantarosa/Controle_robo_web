// Sonda de bancada pra placa de hoverboard.
//
// Existe porque o mega_bridge esconde o que interessa num diagnóstico: o frame
// STATE dele só carrega RPM e bateria, enquanto o feedback da placa traz também
// cmd1/cmd2 (o comando COMO A PLACA O ENTENDEU) e boardTemp. E o test_mega.py
// diz "placa não responde" sem dizer por quê.
//
// SEGURANÇA — o motivo deste sketch ter gatilho:
//   O setup() roda no boot, e gravar reseta a MEGA. Uma versão que acionasse no
//   setup() faria o robô SAIR ANDANDO ao ser flasheado (aconteceu duas vezes na
//   bancada de 2026-09-01). Aqui, parado, ele só ESCUTA: manda speed=0 a 50 Hz e
//   imprime o que a placa responde. Movimento só com tecla explícita.
//
// Teclas (pelo monitor serial, 115200):
//   g  pulso de 0,5 s a speed=+300
//   r  pulso de 0,5 s a speed=-300   (r de reverso)
//   s  imprime uma linha de status agora
//
// O modo parado JÁ É o teste de hall: com speed=0, gire a roda com a mão e olhe
// spdR/spdL. O campo que se mexer diz em qual canal a roda está — e girar na mão
// é o que limpa o errCode que impede a placa de armar (ver BANCADA_HOVER_*.md).
//
//   pio run -t upload --upload-port /dev/ttyACM0
//   pio device monitor -b 115200 --port /dev/ttyACM0

#include <Arduino.h>

static const uint16_t START_FRAME = 0xABCD;
static const uint8_t  FB_SIZE     = 18;
static const int16_t  PULSO_SPEED = 300;
static const uint32_t PULSO_MS    = 500;

#pragma pack(push, 1)
struct Command  { uint16_t start; int16_t steer; int16_t speed; uint16_t checksum; };
struct Feedback {
    uint16_t start;  int16_t cmd1;   int16_t cmd2;
    int16_t  speedR; int16_t speedL; int16_t batVoltage;
    int16_t  boardTemp; uint16_t cmdLed; uint16_t checksum;
};
#pragma pack(pop)

static uint16_t shifter = 0;
static uint8_t  state = 0, buf[FB_SIZE], got = 0;
static Feedback last;
static bool     have = false;
static uint32_t n_ok = 0, n_bad = 0;

// picos da janela corrente (zerados no início de cada pulso)
static int16_t peakR, peakL, bat_min, temp_max;

static void reset_picos() { peakR = 0; peakL = 0; bat_min = 32767; temp_max = -32768; }

static void sendCmd(int16_t steer, int16_t speed) {
    Command c;
    c.start = START_FRAME; c.steer = steer; c.speed = speed;
    c.checksum = (uint16_t)(START_FRAME ^ (uint16_t)steer ^ (uint16_t)speed);
    Serial1.write((const uint8_t*)&c, sizeof(c));
}

static void pump() {
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        if (state == 0) {
            shifter = (uint16_t)((shifter >> 8) | ((uint16_t)b << 8));
            if (shifter == START_FRAME) {
                buf[0] = (uint8_t)(START_FRAME & 0xFF);
                buf[1] = (uint8_t)(START_FRAME >> 8);
                got = 2; state = 1;
            }
            continue;
        }
        buf[got++] = b;
        if (got < FB_SIZE) continue;

        state = 0; shifter = 0;
        Feedback f; memcpy(&f, buf, sizeof(f));
        uint16_t esperado = (uint16_t)(START_FRAME
            ^ (uint16_t)f.cmd1   ^ (uint16_t)f.cmd2
            ^ (uint16_t)f.speedR ^ (uint16_t)f.speedL
            ^ (uint16_t)f.batVoltage ^ (uint16_t)f.boardTemp ^ f.cmdLed);
        if (esperado != f.checksum) { n_bad++; continue; }

        // Faixa de sanidade: um frame corrompido passa no checksum a cada ~65 mil
        // e envenena os picos (já imprimiu "queda de -250 V" uma vez).
        if (f.batVoltage < 1000 || f.batVoltage > 6000) { n_bad++; continue; }

        last = f; have = true; n_ok++;
        if (abs(f.speedR) > abs(peakR)) peakR = f.speedR;
        if (abs(f.speedL) > abs(peakL)) peakL = f.speedL;
        if (f.batVoltage < bat_min)  bat_min  = f.batVoltage;
        if (f.boardTemp  > temp_max) temp_max = f.boardTemp;
    }
}

static void status(const char* tag) {
    if (!have) { Serial.print(tag); Serial.println(F(" (sem feedback da placa)")); return; }
    Serial.print(tag);
    Serial.print(F(" cmd1=")); Serial.print(last.cmd1);
    Serial.print(F(" cmd2=")); Serial.print(last.cmd2);
    Serial.print(F("\tspdR=")); Serial.print(last.speedR);
    Serial.print(F(" spdL=")); Serial.print(last.speedL);
    Serial.print(F("\tbat=")); Serial.print(last.batVoltage / 100.0, 2);
    Serial.print(F("V temp=")); Serial.print(last.boardTemp / 10.0, 1);
    Serial.print(F("C\tok=")); Serial.print(n_ok);
    Serial.print(F(" ruim=")); Serial.println(n_bad);
}

static void pulso(int16_t speed) {
    Serial.print(F("\n>> PULSO speed=")); Serial.print(speed);
    Serial.print(F(" por ")); Serial.print(PULSO_MS); Serial.println(F(" ms"));

    int16_t base_bat = have ? last.batVoltage : 0;
    int16_t base_temp = have ? last.boardTemp : 0;
    reset_picos();

    uint32_t t0 = millis(), t_tx = 0, t_pr = 0;
    while (millis() - t0 < PULSO_MS) {
        if (millis() - t_tx >= 20) { t_tx = millis(); sendCmd(0, speed); }
        pump();
        if (millis() - t_pr >= 100) { t_pr = millis(); status("   "); }
    }
    // inércia: a roda desacelera livre e o hall reporta — o pico costuma
    // aparecer DEPOIS do pulso terminar.
    t0 = millis(); t_tx = 0; t_pr = 0;
    while (millis() - t0 < 1500) {
        if (millis() - t_tx >= 20) { t_tx = millis(); sendCmd(0, 0); }
        pump();
        if (millis() - t_pr >= 250) { t_pr = millis(); status("   coast"); }
    }

    Serial.print(F("   picos: spdR=")); Serial.print(peakR);
    Serial.print(F(" spdL=")); Serial.print(peakL);
    Serial.print(F("  bateria caiu ")); Serial.print((base_bat - bat_min) / 100.0, 2);
    Serial.print(F(" V  temp subiu ")); Serial.print((temp_max - base_temp) / 10.0, 1);
    Serial.println(F(" C"));

    if (abs(peakR) > 3 || abs(peakL) > 3) {
        Serial.println(F("   GIROU."));
    } else if ((base_bat - bat_min) >= 5 || (temp_max - base_temp) >= 3) {
        Serial.println(F("   Corrente circulou sem girar: ordem de fase ou travado."));
    } else {
        Serial.println(F("   Nem corrente nem rotacao: a placa NAO armou."));
        Serial.println(F("   Receita: as DUAS rodas plugadas, e girar ambas na mao"));
        Serial.println(F("   enquanto esta sonda manda zero. Quando o beep mudar, armou."));
    }
    reset_picos();
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    while (!Serial) {}
    delay(300);
    reset_picos();

    Serial.println(F("\n==================================================="));
    Serial.println(F(" SONDA DA PLACA DE HOVERBOARD  (Serial1, 115200)"));
    Serial.println(F(" TX=18  RX=19  GND comum"));
    Serial.println(F("==================================================="));
    Serial.println(F(" Parado: manda speed=0 e so escuta. NADA se move."));
    Serial.println(F(" Gire a roda na mao e olhe spdR/spdL -> testa o hall"));
    Serial.println(F(" e diz em qual canal a roda esta."));
    Serial.println(F(""));
    Serial.println(F(" g = pulso 0,5 s a +300     r = pulso 0,5 s a -300"));
    Serial.println(F(" s = status agora"));
    Serial.println(F(" (g e r MOVEM AS RODAS - deixe no ar)"));
    Serial.println(F("==================================================="));
}

void loop() {
    static uint32_t t_tx = 0, t_pr = 0;

    // Repouso: zero a 50 Hz. Mantém a placa recebendo (ela desarma no timeout)
    // sem pedir torque nenhum.
    if (millis() - t_tx >= 20) { t_tx = millis(); sendCmd(0, 0); }
    pump();
    if (millis() - t_pr >= 1000) { t_pr = millis(); status("[idle]"); }

    if (!Serial.available()) return;
    char c = (char)Serial.read();
    while (Serial.available()) Serial.read();      // descarta o resto da linha

    if      (c == 'g') pulso(PULSO_SPEED);
    else if (c == 'r') pulso(-PULSO_SPEED);
    else if (c == 's') status("[stat]");
}

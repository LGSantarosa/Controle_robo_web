#include <FastLED.h>

// ---- Teste ISOLADO do anel WS2812 na Arduino MEGA ----
// SO' o anel. Nenhum sensor, hoverboard, Wire, SPI ou protocolo.
// Se acender aqui mas nao no firmware completo -> o problema esta' no firmware
// (algo travando o setup antes do anel). Se NAO acender nem aqui -> o problema
// e' fisico: pino 11 da MEGA, fio, contato ou o anel.
//
// LIGACOES:
//   MEGA pino 11 -->  DIN do anel   (mesmo pino do firmware real)
//   Anel 5V      -->  5V do step-down
//   Anel GND     -->  GND comum (junto do GND da MEGA)
//
// LED onboard "L" (pino 13) pisca = este sketch esta' rodando.

constexpr uint8_t DATA_PIN   = 6;    // testando pino 6 (designado no esquematico)
constexpr uint8_t NUM_LEDS   = 24;
constexpr uint8_t BRIGHTNESS = 80;
constexpr uint8_t HB_PIN     = 13;

CRGB leds[NUM_LEDS];

void setup() {
    pinMode(HB_PIN, OUTPUT);

    // NEOPIXEL = mesmo chipset/timing que o firmware real usa (leds.cpp).
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Flash branco no boot: confirma power + escrita basica.
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    delay(500);
    FastLED.clear(true);
    delay(200);
}

void loop() {
    // FASE A — ponto branco girando. Onboard (13) pisca junto.
    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        digitalWrite(HB_PIN, i & 1);
        FastLED.clear();
        leds[i] = CRGB::White;
        FastLED.show();
        delay(60);
    }

    // FASE B — cores solidas no anel inteiro.
    const CRGB cores[3] = { CRGB::Red, CRGB::Green, CRGB::Blue };
    for (uint8_t c = 0; c < 3; ++c) {
        digitalWrite(HB_PIN, c & 1);
        fill_solid(leds, NUM_LEDS, cores[c]);
        FastLED.show();
        delay(600);
    }
}

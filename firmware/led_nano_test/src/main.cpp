#include <FastLED.h>

// ---- Teste de isolamento do anel WS2812 (Arduino Nano) ----
// Objetivo: provar se o "lixo aleatorio" do anel e' culpa da Arduino MEGA.
//   Nano dirige o anel LIMPO  -> problema e' a MEGA (pino/caminho de dados).
//   Nano TAMBEM da' lixo       -> problema e' anel/fiacao/aterramento/power.
//
// LIGACOES (ler com atencao):
//   Nano D6  --(330 ohm em serie, recomendado)-->  DIN do anel
//   Anel 5V  -->  5V do step-down externo  (NAO usar o 5V do Nano: 24 LEDs ~1.4A)
//   Anel GND -->  GND do step-down  E  GND do Nano   (TERRA COMUM — critico!)
//   Mantenha o fio de dados curto.

constexpr uint8_t DATA_PIN   = 6;
constexpr uint8_t NUM_LEDS   = 24;
constexpr uint8_t BRIGHTNESS = 64;   // ~25% p/ nao puxar muita corrente no teste
constexpr uint8_t HB_PIN     = 13;   // LED onboard "L" do Nano = "estou vivo"

CRGB leds[NUM_LEDS];

void setup() {
    pinMode(HB_PIN, OUTPUT);

    // WS2812B, ordem GRB (padrao da maioria dos aneis). Se as cores sairem
    // trocadas mas LIMPAS, troque GRB por RGB — isso NAO e' o bug que buscamos.
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Flash branco curto no boot: confirma power + escrita basica.
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    delay(400);
    FastLED.clear(true);
    delay(200);
}

void loop() {
    // FASE A — ponto unico andando pelo anel. O LED onboard (13) pisca a cada
    // passo: se ELE pisca, o chip esta' rodando — ai' anel apagado = caminho de
    // dados/anel. Se nem o onboard pisca, o chip nao esta' executando o sketch.
    // Um unico ponto limpo girando = dados integros; pontos extras = corrompido.
    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        digitalWrite(HB_PIN, i & 1);
        FastLED.clear();
        leds[i] = CRGB::White;
        FastLED.show();
        delay(60);
    }

    // FASE B — cores solidas: confirma os 24 enderecaveis e uniformes.
    const CRGB cores[3] = { CRGB::Red, CRGB::Green, CRGB::Blue };
    for (uint8_t c = 0; c < 3; ++c) {
        digitalWrite(HB_PIN, c & 1);
        fill_solid(leds, NUM_LEDS, cores[c]);
        FastLED.show();
        delay(600);
    }
}

#include "io_signals.h"

namespace io_signals {

namespace {
// Nível elétrico que corresponde a "relé ligado", dada a polaridade do módulo.
constexpr uint8_t relayLevel(bool on) {
    return (on != RELAY_ACTIVE_LOW) ? HIGH : LOW;
}
}  // namespace

void begin() {
    // A ordem importa: digitalWrite ANTES do pinMode liga o pull-up interno e já
    // segura a linha no nível de "apagado" enquanto o pino ainda é entrada. Com
    // o pinMode primeiro, o pino passa por LOW e o relê dá um clique (a luz
    // pisca) a cada reset da MEGA — e ela reseta toda vez que a Pi abre a serial.
    digitalWrite(PIN_RELAY, relayLevel(false));
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, relayLevel(false));

    pinMode(PIN_LED,   OUTPUT);
    pinMode(PIN_BTN,   INPUT_PULLUP);
    digitalWrite(PIN_LED,   LOW);
}

void setRelay(bool on)     { digitalWrite(PIN_RELAY, relayLevel(on)); }
void setMarkerLed(bool on) { digitalWrite(PIN_LED,   on ? HIGH : LOW); }
bool readButton()          { return digitalRead(PIN_BTN) == LOW; }

}  // namespace io_signals

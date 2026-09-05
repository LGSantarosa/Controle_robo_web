#pragma once
#include <Arduino.h>

namespace io_signals {

constexpr uint8_t PIN_RELAY = 7;
constexpr uint8_t PIN_LED   = 8;
constexpr uint8_t PIN_BTN   = 9;

// Polaridade do módulo de relê da luz. Medido no robô em 2026-09-03: com o pino
// em LOW a bobina fica ENERGIZADA e a luz acende — é módulo ativo-BAIXO, como a
// maioria dos que trazem optoacoplador. Sem esta constante a luz nascia ACESA
// (o begin() deixava o pino em LOW) e só apagava com um comando explícito:
// bateria queimando à toa, e a luz é forte.
// Trocou o módulo por um ativo-ALTO? Só virar isto pra false.
constexpr bool RELAY_ACTIVE_LOW = true;

void begin();
void setRelay(bool on);
void setMarkerLed(bool on);
bool readButton();

}  // namespace io_signals

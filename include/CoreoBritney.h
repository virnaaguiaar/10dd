#ifndef COREO_BRITNEY_H
#define COREO_BRITNEY_H

#include "Movimento.h"
#include "Configs.h"

struct CoreografiaEtapa {
  int velocidadeX;
  int velocidadeY;
  unsigned long duracaoMs;
  uint8_t leds;
};

static const CoreografiaEtapa COREOGRAFIA[] = {
  { 9,  0, 3000, 0b11111111},
  {-9,  0, 2000, 0b11111111},
  { 0,  4, 1000, 0b10100110},
  { 0, -4, 1000, 0b00101110},
  { 5,  5, 3000, 0b01000011},
  { 5, -5, 1000, 0b00011110},
  {-5,  5, 3000, 0b11000001},
  {-5, -5, 1000, 0b00011110},
  { 9,  0, 3000, 0b11111111},
  {-9,  0, 2000, 0b11111111},
  {-5, -5, 4000, 0b00011110},
  { 9,  0, 3000, 0b11111111}

};

static const size_t COREOGRAFIA_ETAPAS = sizeof(COREOGRAFIA) / sizeof(COREOGRAFIA[0]);
static size_t etapaAtual = 0;
static unsigned long inicioDaEtapa = 0;

inline void aplicarLedsCoreografia(uint8_t estado) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    digitalWrite(led[i], (estado & (1 << i)) ? HIGH : LOW);
  }
}

inline void iniciarEtapaCoreografia() {
  const CoreografiaEtapa& etapa = COREOGRAFIA[etapaAtual];
  aplicarLedsCoreografia(etapa.leds);
  PWM(etapa.velocidadeX, etapa.velocidadeY);
  inicioDaEtapa = millis();
}

inline void iniciarCoreografia() {
  if (coreografiaEmExecucao) return;

  Serial.println("🎬 Iniciando coreografia...");
  coreografiaEmExecucao = true;
  etapaAtual = 0;
  iniciarEtapaCoreografia();
}

inline void pararCoreografia() {
  if (!coreografiaEmExecucao) return;

  coreografiaEmExecucao = false;
  PWM(0, 0);
  desligaLeds();
  Serial.println("🛑 Coreografia interrompida");
}

// Called on every main-loop iteration; it does not use delay(), keeping MQTT,
// OTA, stop, and manual-control commands responsive.
inline void atualizarCoreografia() {
  if (!coreografiaEmExecucao) return;

  const CoreografiaEtapa& etapa = COREOGRAFIA[etapaAtual];
  if (millis() - inicioDaEtapa < etapa.duracaoMs) return;

  ++etapaAtual;
  if (etapaAtual >= COREOGRAFIA_ETAPAS) {
    coreografiaEmExecucao = false;
    PWM(0, 0);
    desligaLeds();
    Serial.println("🏁 Coreografia finalizada!");
    return;
  }

  iniciarEtapaCoreografia();
}

// Compatibility with existing callers.
inline void coreo() { iniciarCoreografia(); }

#endif

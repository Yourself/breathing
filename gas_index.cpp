#include "gas_index.h"

void GasIndexStateMachine::update(float temp, int rhum, unsigned long millis) {
  if (millis - prevUpdate_ < updateInterval)
    return;

  uint16_t rhumComp = static_cast<uint16_t>(rhum * 65535 / 100);
  uint16_t tempComp = static_cast<uint16_t>((temp + 45) * 65535 / 175);

  uint16_t srawVoc;
  uint16_t srawNox;

  if (conditioningCountdown_ > 0) {
    if (sgp41_.executeConditioning(rhumComp, tempComp, srawVoc)) {
      voc_ = -1;
      nox_ = -1;
    } else {
      voc_ = voc_algo_.process(srawVoc);
      --conditioningCountdown_;
    }
  } else {
    if (sgp41_.measureRawSignals(rhumComp, tempComp, srawVoc, srawNox)) {
      voc_ = -1;
      nox_ = -1;
    } else {
      voc_ = voc_algo_.process(srawVoc);
      nox_ = nox_algo_.process(srawNox);
    }
  }
  prevUpdate_ += updateInterval;
}
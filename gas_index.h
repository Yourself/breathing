#ifndef GAS_INDEX_H_
#define GAS_INDEX_H_

#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <VOCGasIndexAlgorithm.h>

class GasIndexStateMachine {
public:
  static const int updateInterval = 1000;

  int voc() const { return voc_; }
  int nox() const { return nox_; }

  bool voc_available() const { return voc_ >= 0; }
  bool nox_available() const { return nox_ >= 0; }

  void begin(TwoWire &i2cbus) { sgp41_.begin(i2cbus); }

  void update(float temp, float rhum, unsigned long millis);

private:
  SensirionI2CSgp41 sgp41_;
  VOCGasIndexAlgorithm voc_algo_;
  NOxGasIndexAlgorithm nox_algo_;

  unsigned long prevUpdate_ = 0;

  int voc_ = -1;
  int nox_ = -1;

  int conditioningCountdown_ = 9;
};

#endif
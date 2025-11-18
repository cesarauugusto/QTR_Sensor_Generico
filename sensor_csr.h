#ifndef SENSOR_CSR_H
#define SENSOR_CSR_H

#include <Arduino.h>

class sensor_csr
{
public:
  sensor_csr();

  // Configuração básica
  void setTypeAnalog(); // mantido só por compatibilidade
  void setSensorPins(const uint8_t *pins, uint8_t numSensors);
  void setSamplesPerSensor(uint8_t samples);

  // Calibração e leitura
  void calibrate(uint16_t ms, bool useAmbient);
  void readCalibrated(uint16_t *dest, bool useAmbient, bool useEmitter);

  // Calcula erro discreto (-7000..+7000) e preenche bits ("00011000", etc.)
  int ErroSensor(char *bitsOut,
                 uint16_t limiarBranco,
                 uint16_t limiarPreto,
                 bool useAmbient,
                 int lastSteps);

  // retorna true se todos os bits forem '0' (GAP: "00000000")
  bool GapSensor(const char *bits) const;

private:
  uint8_t _numSensors;
  const uint8_t *_pins;
  uint8_t _samples;
  bool _isAnalog;

  uint16_t *_minVal;
  uint16_t *_maxVal;

  void allocCalibArrays();
};

#endif

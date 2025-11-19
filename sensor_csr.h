#ifndef SENSOR_CSR_H
#define SENSOR_CSR_H

#include <Arduino.h>

// Número máximo de sensores suportados
#define SENSOR_CSR_MAX_SENSORS 8

class sensor_csr
{
public:
  sensor_csr();

  // Sensores analógicos genéricos (QTR, TCRT5000, etc.)
  void setTypeAnalog(); // Mantido por compatibilidade
  void setSensorPins(const uint8_t *pins, uint8_t count);
  void setSamplesPerSensor(uint8_t samples);
  void setEmitterPin(uint8_t pin);

  // Calibração automática em 'calibrateTimeMs' milissegundos
  void calibrate(uint16_t calibrateTimeMs, bool print = false);

  // Leituras
  void readRaw(uint16_t *values);        // 0..1023
  void readCalibrated(uint16_t *values); // 0..1000 (normalizado)

  // Definição opcional de limiares binários (branco/preto)
  void setBinaryThresholds(uint16_t limiarBranco, uint16_t limiarPreto);

  // Lê padrão binário usando limiares internos
  void readBinary(char *bitsOut);

  // === Função principal usada no robô ===
  // Usa limiares passados e retorna erro discreto em múltiplos de 1000.
  // Faixa típica: -7000 .. +7000
  // O padrão 0/1 fica guardado internamente e pode ser lido com getBits().
  int ErroSensor(int limiarBranco, int limiarPreto);

  // Detecta GAP (tracejado): true se último padrão foi "00000000"
  bool gapDetection() const;

  // Acesso ao último padrão interno (para debug Serial.print)
  const char *getBits() const { return _bits; }

  // Cálculo do erro a partir de um padrão 0/1
  int computeErrorFromBits(const char *bits = nullptr) const;

private:
  uint8_t _pins[SENSOR_CSR_MAX_SENSORS];
  uint8_t _numSensors;
  uint8_t _samplesPerSensor;

  int16_t _minVal[SENSOR_CSR_MAX_SENSORS];
  int16_t _maxVal[SENSOR_CSR_MAX_SENSORS];

  uint8_t _emitterPin;
  bool _hasEmitter;

  uint16_t _whiteThresh;
  uint16_t _blackThresh;

  // Último padrão binário lido
  char _bits[SENSOR_CSR_MAX_SENSORS + 1];

  void ensureCalibratedRange();
};

#endif

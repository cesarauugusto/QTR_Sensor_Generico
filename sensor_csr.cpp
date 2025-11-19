#include "sensor_csr.h"

sensor_csr::sensor_csr()
  : _numSensors(0),
    _samplesPerSensor(5),
    _emitterPin(255),
    _hasEmitter(false),
    _whiteThresh(600),
    _blackThresh(900)
{
  for (uint8_t i = 0; i < SENSOR_CSR_MAX_SENSORS; i++) {
    _pins[i]    = 255;
    _minVal[i]  = 1023;
    _maxVal[i]  = 0;
    _bits[i]    = '0';
  }
  _bits[SENSOR_CSR_MAX_SENSORS] = '\0';
}

void sensor_csr::setTypeAnalog() {
  // Mantido apenas por compatibilidade.
}

void sensor_csr::setSensorPins(const uint8_t *pins, uint8_t count) {
  if (count > SENSOR_CSR_MAX_SENSORS) count = SENSOR_CSR_MAX_SENSORS;
  _numSensors = count;
  for (uint8_t i = 0; i < count; i++) {
    _pins[i]   = pins[i];
    _minVal[i] = 1023;
    _maxVal[i] = 0;
  }
}

void sensor_csr::setSamplesPerSensor(uint8_t samples) {
  if (samples == 0) samples = 1;
  _samplesPerSensor = samples;
}

void sensor_csr::setEmitterPin(uint8_t pin) {
  _emitterPin = pin;
  _hasEmitter = true;
  pinMode(_emitterPin, OUTPUT);
  digitalWrite(_emitterPin, HIGH);
}

void sensor_csr::readRaw(uint16_t *values) {
  if (_hasEmitter) {
    digitalWrite(_emitterPin, HIGH);
    delayMicroseconds(200);
  }

  for (uint8_t i = 0; i < _numSensors; i++) {
    uint32_t soma = 0;
    for (uint8_t s = 0; s < _samplesPerSensor; s++) {
      soma += analogRead(_pins[i]);
    }
    values[i] = (uint16_t)(soma / _samplesPerSensor);
  }
}

void sensor_csr::ensureCalibratedRange() {
  for (uint8_t i = 0; i < _numSensors; i++) {
    if (_maxVal[i] <= _minVal[i]) {
      _minVal[i] = 0;
      _maxVal[i] = 1023;
    }
  }
}

void sensor_csr::calibrate(uint16_t calibrateTimeMs, bool print) {
  uint32_t start = millis();
  uint16_t valores[SENSOR_CSR_MAX_SENSORS];

  while ((millis() - start) < calibrateTimeMs) {
    readRaw(valores);
    for (uint8_t i = 0; i < _numSensors; i++) {
      if (valores[i] < _minVal[i]) _minVal[i] = valores[i];
      if (valores[i] > _maxVal[i]) _maxVal[i] = valores[i];
    }
    if (print) {
      for (uint8_t i = 0; i < _numSensors; i++) {
        Serial.print(valores[i]);
        Serial.print('\t');
      }
      Serial.println();
    }
    delay(5);
  }
  ensureCalibratedRange();
}

void sensor_csr::readCalibrated(uint16_t *values) {
  uint16_t raw[SENSOR_CSR_MAX_SENSORS];
  readRaw(raw);

  for (uint8_t i = 0; i < _numSensors; i++) {
    int16_t minv = _minVal[i];
    int16_t maxv = _maxVal[i];
    if (maxv == minv) {
      values[i] = 0;
    } else {
      long val = (long)(raw[i] - minv) * 1000L / (long)(maxv - minv);
      if (val < 0)    val = 0;
      if (val > 1000) val = 1000;
      values[i] = (uint16_t)val;
    }
  }
}

void sensor_csr::setBinaryThresholds(uint16_t limiarBranco, uint16_t limiarPreto) {
  _whiteThresh = limiarBranco;
  _blackThresh = limiarPreto;
}

void sensor_csr::readBinary(char *bitsOut) {
  uint16_t vals[SENSOR_CSR_MAX_SENSORS];
  readCalibrated(vals);

  for (uint8_t i = 0; i < _numSensors; i++) {
    char bit;
    if (vals[i] >= _blackThresh)       bit = '1';
    else if (vals[i] <= _whiteThresh)  bit = '0';
    else                               bit = '0';  // zona cinza → branco
    _bits[i] = bit;
    if (bitsOut) bitsOut[i] = bit;
  }
  _bits[_numSensors] = '\0';
  if (bitsOut) bitsOut[_numSensors] = '\0';
}

// Mapeamento discreto que você definiu (erros -7..+7 * 1000):
//
// 00000001 ->  7
// 00000011 ->  6
// 00000111 ->  5
// 00000110 ->  4
// 00001110 ->  3
// 00001100 ->  2
// 00011100 ->  1
// 00011000 ->  0
// 00111000 -> -1
// 00110000 -> -2
// 01110000 -> -3
// 01100000 -> -4
// 11100000 -> -5
// 11000000 -> -6
// 10000000 -> -7
int sensor_csr::computeErrorFromBits(const char *bits) const {
  const char *b = bits ? bits : _bits;

  if (_numSensors == 8) {
    auto eq = [&](const char *pat) -> bool {
      for (uint8_t i = 0; i < 8; i++) {
        if (b[i] != pat[i]) return false;
      }
      return true;
    };

    if (eq("00011000")) return  0 * 1000;
    if (eq("00011100")) return  1 * 1000;
    if (eq("00001100")) return  2 * 1000;
    if (eq("00001110")) return  3 * 1000;
    if (eq("00000110")) return  4 * 1000;
    if (eq("00000111")) return  5 * 1000;
    if (eq("00000011")) return  6 * 1000;
    if (eq("00000001")) return  7 * 1000;

    if (eq("00111000")) return -1 * 1000;
    if (eq("00110000")) return -2 * 1000;
    if (eq("01110000")) return -3 * 1000;
    if (eq("01100000")) return -4 * 1000;
    if (eq("11100000")) return -5 * 1000;
    if (eq("11000000")) return -6 * 1000;
    if (eq("10000000")) return -7 * 1000;
  }

  // Fallback: centro de massa
  long somaPeso = 0;
  long somaAtivo = 0;
  for (uint8_t i = 0; i < _numSensors; i++) {
    if (b[i] == '1') {
      int peso = map(i, 0, _numSensors - 1, -7, 7);
      somaPeso  += peso;
      somaAtivo += 1;
    }
  }
  if (somaAtivo == 0) {
    return 0;
  }
  long media = somaPeso / somaAtivo;
  return (int)(media * 1000);
}

int sensor_csr::ErroSensor(int limiarBranco, int limiarPreto) {
  _whiteThresh = limiarBranco;
  _blackThresh = limiarPreto;

  // Atualiza _bits internamente
  readBinary(nullptr);

  // Calcula erro discreto usando _bits
  return computeErrorFromBits(_bits);
}

bool sensor_csr::gapDetection() const {
  if (_numSensors == 0) return false;
  for (uint8_t i = 0; i < _numSensors; i++) {
    if (_bits[i] == '1') return false;
  }
  return true;  // todos zero → GAP
}

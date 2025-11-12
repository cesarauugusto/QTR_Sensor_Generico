#include "sensor_csr.h"

void sensor_csr::setSensorPins(const uint8_t *pins, uint8_t count) {
  _numSensors = (count > MAX_SENSORS) ? MAX_SENSORS : count;
  for (uint8_t i=0;i<_numSensors;i++) _pins[i] = pins[i];
}

void sensor_csr::setEmitterPin(uint8_t pin) {
  _hasEmitter = true; _emitterPin = pin;
  pinMode(_emitterPin, OUTPUT);
  digitalWrite(_emitterPin, LOW);
}

void sensor_csr::setSamplesPerSensor(uint8_t s) {
  _samplesPerSensor = (s==0)?1:s;
}

uint16_t sensor_csr::readOneAnalog(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t k=0;k<_samplesPerSensor;k++) acc += analogRead(pin);
  return (uint16_t)(acc / _samplesPerSensor);
}

uint16_t sensor_csr::readOneRC(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);

  pinMode(pin, INPUT);
  uint16_t count = 0;
  const uint16_t timeout = 3000;
  while (digitalRead(pin) == HIGH && count < timeout) {
    count++;
    delayMicroseconds(1);
  }
  return count;
}

void sensor_csr::emitter(bool on) {
  if (_hasEmitter) digitalWrite(_emitterPin, on?HIGH:LOW);
}

void sensor_csr::read(uint16_t *rawOut, bool emitterOn) {
  if (_numSensors == 0) return;
  emitter(emitterOn);
  delayMicroseconds(200);
  for (uint8_t i=0;i<_numSensors;i++) {
    uint16_t v = (_type == TYPE_ANALOG) ? readOneAnalog(_pins[i]) : readOneRC(_pins[i]);
    rawOut[i] = v;
  }
}

void sensor_csr::calibrate(uint16_t nIters, bool withAmbient) {
  if (_numSensors == 0) return;
  uint16_t v;

  for (uint16_t it=0; it<nIters; it++) {
    emitter(true);
    delayMicroseconds(200);
    for (uint8_t i=0;i<_numSensors;i++) {
      v = (_type == TYPE_ANALOG) ? readOneAnalog(_pins[i]) : readOneRC(_pins[i]);
      if (v < _minOn[i]) _minOn[i] = v;
      if (v > _maxOn[i]) _maxOn[i] = v;
    }
  }

  if (withAmbient && _hasEmitter) {
    for (uint16_t it=0; it<nIters; it++) {
      emitter(false);
      delayMicroseconds(200);
      for (uint8_t i=0;i<_numSensors;i++) {
        v = (_type == TYPE_ANALOG) ? readOneAnalog(_pins[i]) : readOneRC(_pins[i]);
        if (v < _minOff[i]) _minOff[i] = v;
        if (v > _maxOff[i]) _maxOff[i] = v;
      }
    }
  } else {
    for (uint8_t i=0;i<_numSensors;i++) { _minOff[i]=0; _maxOff[i]=0; }
  }
}

void sensor_csr::readCalibrated(uint16_t *calOut, bool whiteLine, bool useAmbient) {
  if (_numSensors == 0) return;
  uint16_t rawOn[MAX_SENSORS];
  uint16_t rawOff[MAX_SENSORS];

  read(rawOn, true);

  bool ambientAvailable = useAmbient && _hasEmitter && (_maxOff[0] != 0 || _minOff[0] != 0);
  if (ambientAvailable) read(rawOff, false);

  for (uint8_t i=0;i<_numSensors;i++) {
    int32_t vEff;
    if (ambientAvailable) {
      vEff = (int32_t)rawOn[i] - (int32_t)rawOff[i];
      int32_t loEff = (int32_t)_minOn[i] - (int32_t)_maxOff[i];
      int32_t hiEff = (int32_t)_maxOn[i] - (int32_t)_minOff[i];
      if (hiEff <= loEff) hiEff = loEff + 1;
      int32_t cal = ((vEff - loEff) * 1000L) / (hiEff - loEff);
      cal = clampu16(cal, 0, 1000);
      calOut[i] = whiteLine ? (1000 - (uint16_t)cal) : (uint16_t)cal;
    } else {
      vEff = rawOn[i];
      int32_t lo = _minOn[i], hi = _maxOn[i];
      if (hi <= lo) hi = lo + 1;
      int32_t cal = ((vEff - lo) * 1000L) / (hi - lo);
      cal = clampu16(cal, 0, 1000);
      calOut[i] = whiteLine ? (1000 - (uint16_t)cal) : (uint16_t)cal;
    }
  }
}

uint16_t sensor_csr::readLineBlack(uint16_t *optRawOut, bool useAmbient) {
  if (_numSensors == 0) return 0;
  uint16_t cal[MAX_SENSORS];
  if (optRawOut) read(optRawOut, true);
  readCalibrated(cal, false, useAmbient);

  uint32_t num = 0, den = 0;
  for (uint8_t i=0;i<_numSensors;i++) {
    uint16_t v = cal[i];
    num += (uint32_t)v * (uint32_t)(i * 1000);
    den += v;
  }

  if (den == 0) {
    uint16_t center = (_numSensors - 1) * 1000 / 2;
    _lastPosition = (_lastPosition < center) ? 0 : (uint16_t)((_numSensors - 1) * 1000);
    return _lastPosition;
  }

  _lastPosition = (uint16_t)(num / den);
  return _lastPosition;
}

uint16_t sensor_csr::readLineWhite(uint16_t *optRawOut, bool useAmbient) {
  if (_numSensors == 0) return 0;
  uint16_t cal[MAX_SENSORS];
  if (optRawOut) read(optRawOut, true);
  readCalibrated(cal, true, useAmbient);

  uint32_t num = 0, den = 0;
  for (uint8_t i=0;i<_numSensors;i++) {
    uint16_t v = cal[i];
    num += (uint32_t)v * (uint32_t)(i * 1000);
    den += v;
  }

  if (den == 0) {
    uint16_t center = (_numSensors - 1) * 1000 / 2;
    _lastPosition = (_lastPosition < center) ? 0 : (uint16_t)((_numSensors - 1) * 1000);
    return _lastPosition;
  }

  _lastPosition = (uint16_t)(num / den);
  return _lastPosition;
}

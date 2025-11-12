#ifndef SENSOR_CSR_H
#define SENSOR_CSR_H

#include <Arduino.h>

class sensor_csr {
public:
  static const uint8_t MAX_SENSORS = 16;

  enum Type { TYPE_ANALOG = 0, TYPE_RC = 1 };

  sensor_csr()
  : _type(TYPE_ANALOG),
    _numSensors(0),
    _samplesPerSensor(4),
    _hasEmitter(false),
    _emitterPin(255),
    _lastPosition(0)
  {
    for (uint8_t i=0;i<MAX_SENSORS;i++){
      _pins[i] = 255;
      _minOn[i] = 1023; _maxOn[i] = 0;
      _minOff[i]= 1023; _maxOff[i]= 0;
    }
  }

  // === Configuração ===
  void setTypeAnalog() { _type = TYPE_ANALOG; }
  void setTypeRC()     { _type = TYPE_RC;     }
  void setSensorPins(const uint8_t *pins, uint8_t count);
  void setEmitterPin(uint8_t pin);
  void setSamplesPerSensor(uint8_t s);

  // === Calibração ===
  void calibrate(uint16_t nIters = 10, bool withAmbient=false);

  // === Leitura ===
  void read(uint16_t *rawOut, boo*_

/*
===========================================================
  Biblioteca: sensor_csr
  Autor:      César Augusto Victor
  Descrição:  Leitura e processamento para sensores
               QTR genéricos (não-Pololu).
  Licença:    MIT License
===========================================================
*/

#ifndef SENSOR_CSR_H
#define SENSOR_CSR_H

#include <Arduino.h>

class sensor_csr {

public:

    sensor_csr();
    void setTypeAnalog();  
    void setSensorPins(const uint8_t *pins, uint8_t count);
    void setSamplesPerSensor(uint8_t samples);

    void calibrate(uint16_t tempoMS, bool printValores);
    void readRaw(uint16_t *dest);
    void readCalibrated(uint16_t *dest, bool blackIsHigh, bool printDebug);

    int ErroSensor(char *bitsOut,
                   uint16_t limiarBranco,
                   uint16_t limiarPreto,
                   bool usarAmbiente,
                   int erroStepsAnterior);

private:

    uint8_t _pins[16];
    uint8_t _numSensores;
    uint8_t _samples;

    uint16_t _calMin[16];
    uint16_t _calMax[16];

    bool _analogMode;
};

#endif

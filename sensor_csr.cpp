#include "sensor_csr.h"

/* ----------------------------------------------------------
   Construtor
---------------------------------------------------------- */
sensor_csr::sensor_csr(){
    _numSensores = 0;
    _samples = 4;
    _analogMode = true;

    for (int i = 0; i < 16; i++){
        _calMin[i] = 1023;
        _calMax[i] = 0;
        _pins[i] = 0;
    }
}

/* ----------------------------------------------------------
   Configura sensores analógicos
---------------------------------------------------------- */
void sensor_csr::setTypeAnalog(){
    _analogMode = true;
}

/* ----------------------------------------------------------
   Define pinos dos sensores
---------------------------------------------------------- */
void sensor_csr::setSensorPins(const uint8_t *pins, uint8_t count){
    _numSensores = count;
    for (int i = 0; i < count; i++){
        _pins[i] = pins[i];
    }
}

/* ----------------------------------------------------------
   Define número de amostras por sensor
---------------------------------------------------------- */
void sensor_csr::setSamplesPerSensor(uint8_t samples){
    _samples = samples;
}

/* ----------------------------------------------------------
   Lê valores crus (raw) dos sensores
---------------------------------------------------------- */
void sensor_csr::readRaw(uint16_t *dest){
    for (int i = 0; i < _numSensores; i++){
        uint32_t soma = 0;
        for (int k = 0; k < _samples; k++){
            soma += analogRead(_pins[i]);
        }
        dest[i] = soma / _samples;
    }
}

/* ----------------------------------------------------------
   Calibração — mede min/max para normalizar
---------------------------------------------------------- */
void sensor_csr::calibrate(uint16_t tempoMS, bool printValores){
    uint16_t raw[16];
    uint32_t t0 = millis();
    while (millis() - t0 < tempoMS){
        readRaw(raw);
        for (int i = 0; i < _numSensores; i++){
            if (raw[i] < _calMin[i]) _calMin[i] = raw[i];
            if (raw[i] > _calMax[i]) _calMax[i] = raw[i];
        }
        if (printValores){
            for (int i = 0; i < _numSensores; i++){
                Serial.print(raw[i]); Serial.print("\t");
            }
            Serial.println();
        }
    }
}

/* ----------------------------------------------------------
   Leitura calibrada (0–1000)
---------------------------------------------------------- */
void sensor_csr::readCalibrated(uint16_t *dest, bool blackIsHigh, bool printDebug){
    uint16_t raw[16];
    readRaw(raw);

    for (int i = 0; i < _numSensores; i++){
        uint16_t mn = _calMin[i];
        uint16_t mx = _calMax[i];
        uint16_t v  = raw[i];

        if (mx <= mn) dest[i] = 0;
        else          dest[i] = (uint32_t)(v - mn) * 1000 / (mx - mn);

        if (printDebug){
            Serial.print(dest[i]);
            Serial.print("\t");
        }
    }
    if (printDebug) Serial.println();
}

/* ----------------------------------------------------------
   ErroSensor()
   Retorna erro de -4000 a +4000 e gera bits ("00011000")
---------------------------------------------------------- */
int sensor_csr::ErroSensor(char *bitsOut,
                           uint16_t limiarBranco,
                           uint16_t limiarPreto,
                           bool usarAmbiente,
                           int erroStepsAnterior)
{
    uint16_t cal[16];
    readCalibrated(cal, false, false);

    int somaIdx = 0;
    int somaBit = 0;

    for (int i = 0; i < _numSensores; i++){
        uint16_t v = cal[i];

        uint8_t bit = 0;
        if (v > limiarPreto) bit = 1;
        else if (v < limiarBranco) bit = 0;
        else bit = 0; // zona cinza

        bitsOut[i] = bit ? '1' : '0';

        if (bit){
            somaIdx += i;
            somaBit++;
        }
    }

    bitsOut[_numSensores] = '\0';

    if (somaBit == 0){
        if (erroStepsAnterior > 0) return +4000;
        if (erroStepsAnterior < 0) return -4000;
        return 0;
    }

    float avg = (float)somaIdx / (float)somaBit;
    int steps = (int)(avg - ((_numSensores - 1) / 2.0f));

    if (steps >  4) steps =  4;
    if (steps < -4) steps = -4;

    return steps * 1000;
}

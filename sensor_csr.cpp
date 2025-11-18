#include "sensor_csr.h"
#include <string.h> // strcmp

sensor_csr::sensor_csr()
    : _numSensors(0),
      _pins(NULL),
      _samples(4),
      _isAnalog(true),
      _minVal(NULL),
      _maxVal(NULL)
{
}

void sensor_csr::setTypeAnalog()
{
  _isAnalog = true;
}

void sensor_csr::setSensorPins(const uint8_t *pins, uint8_t numSensors)
{
  _pins = pins;
  _numSensors = numSensors;
  allocCalibArrays();
}

void sensor_csr::setSamplesPerSensor(uint8_t samples)
{
  _samples = samples;
}

void sensor_csr::allocCalibArrays()
{
  if (_numSensors == 0)
    return;

  if (_minVal)
    free(_minVal);
  if (_maxVal)
    free(_maxVal);

  _minVal = (uint16_t *)malloc(_numSensors * sizeof(uint16_t));
  _maxVal = (uint16_t *)malloc(_numSensors * sizeof(uint16_t));

  if (!_minVal || !_maxVal)
    return;

  for (uint8_t i = 0; i < _numSensors; i++)
  {
    _minVal[i] = 1023;
    _maxVal[i] = 0;
  }
}

void sensor_csr::calibrate(uint16_t ms, bool useAmbient)
{
  (void)useAmbient; // não usamos por enquanto

  if (!_pins || _numSensors == 0)
    return;

  unsigned long start = millis();
  while ((uint16_t)(millis() - start) < ms)
  {
    for (uint8_t i = 0; i < _numSensors; i++)
    {
      long soma = 0;
      for (uint8_t s = 0; s < _samples; s++)
      {
        soma += analogRead(_pins[i]);
      }
      uint16_t raw = (uint16_t)(soma / _samples);

      if (raw < _minVal[i])
        _minVal[i] = raw;
      if (raw > _maxVal[i])
        _maxVal[i] = raw;
    }
    delay(5);
  }
}

void sensor_csr::readCalibrated(uint16_t *dest, bool useAmbient, bool useEmitter)
{
  (void)useAmbient;
  (void)useEmitter;

  if (!_pins || _numSensors == 0 || !dest)
    return;

  for (uint8_t i = 0; i < _numSensors; i++)
  {
    long soma = 0;
    for (uint8_t s = 0; s < _samples; s++)
    {
      soma += analogRead(_pins[i]);
    }
    uint16_t raw = (uint16_t)(soma / _samples);

    uint16_t minv = _minVal ? _minVal[i] : 0;
    uint16_t maxv = _maxVal ? _maxVal[i] : 1023;

    if (maxv <= minv)
    {
      dest[i] = 0;
    }
    else
    {
      long v = ((long)(raw - minv) * 1000L) / (long)(maxv - minv);
      if (v < 0)
        v = 0;
      if (v > 1000)
        v = 1000;
      dest[i] = (uint16_t)v;
    }
  }
}

// --------------------- ErroSensor ---------------------
int sensor_csr::ErroSensor(char *bitsOut,
                           uint16_t limiarBranco,
                           uint16_t limiarPreto,
                           bool useAmbient,
                           int lastSteps)
{
  (void)useAmbient;

  if (!_pins || _numSensors == 0)
  {
    if (bitsOut)
      bitsOut[0] = '\0';
    return lastSteps * 1000;
  }

  // Limita para 16 sensores por segurança
  uint8_t n = _numSensors;
  if (n > 16)
    n = 16;

  uint16_t cal[16];
  readCalibrated(cal, false, true);

  int count1 = 0;

  for (uint8_t i = 0; i < n; i++)
  {
    uint16_t v = cal[i];
    char bit = '0';

    if (v >= limiarPreto)
      bit = '1'; // preto
    else if (v <= limiarBranco)
      bit = '0'; // branco
    else
      bit = '0'; // zona cinza => branco

    if (bitsOut)
      bitsOut[i] = bit;
    if (bit == '1')
      count1++;
  }

  if (bitsOut)
    bitsOut[n] = '\0';

  // Nenhum sensor viu linha → mantemos lastSteps (gap ou perda total)
  if (count1 == 0)
  {
    return lastSteps * 1000;
  }

  int steps = 0;

  // ----------------- Tabela para 8 sensores -----------------
  if (n == 8 && bitsOut)
  {
    struct PatternMap
    {
      const char *pattern;
      int steps;
    };

    static const PatternMap table[] = {
        {"00000001", 7},
        {"00000011", 6},
        {"00000111", 5},
        {"00000110", 4},
        {"00001110", 3},
        {"00001100", 2},
        {"00011100", 1},
        {"00011000", 0},
        {"00111000", -1},
        {"00110000", -2},
        {"01110000", -3},
        {"01100000", -4},
        {"11100000", -5},
        {"11000000", -6},
        {"10000000", -7}};

    bool matched = false;
    for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); i++)
    {
      if (strcmp(bitsOut, table[i].pattern) == 0)
      {
        steps = table[i].steps;
        matched = true;
        break;
      }
    }

    if (!matched)
    {
      // Fallback: centro de massa
      float somaIdx = 0.0f;
      for (uint8_t i = 0; i < n; i++)
      {
        if (bitsOut[i] == '1')
          somaIdx += i;
      }
      float avg = somaIdx / (float)count1; // 0..7
      float center = 3.5f;
      float rel = avg - center; // -3.5..+3.5
      float s = rel * 2.0f;     // ~ -7..+7

      if (s > 7.0f)
        s = 7.0f;
      if (s < -7.0f)
        s = -7.0f;

      // Arredondinho simples
      if (s >= 0)
        steps = (int)(s + 0.5f);
      else
        steps = (int)(s - 0.5f);
    }
  }
  else
  {
    // Genérico pra outro número de sensores
    float somaIdx = 0.0f;
    for (uint8_t i = 0; i < n; i++)
    {
      if (!bitsOut || bitsOut[i] == '1')
        somaIdx += i;
    }
    float avg = somaIdx / (float)count1;
    float center = (n - 1) / 2.0f;
    float rel = avg - center;
    float s = rel * 2.0f;

    if (s > 7.0f)
      s = 7.0f;
    if (s < -7.0f)
      s = -7.0f;

    if (s >= 0)
      steps = (int)(s + 0.5f);
    else
      steps = (int)(s - 0.5f);
  }

  return steps * 1000;
}

// --------------------- GapSensor ---------------------
bool sensor_csr::GapSensor(const char *bits) const
{
  if (!bits)
    return false;
  for (int i = 0; bits[i] != '\0'; i++)
  {
    if (bits[i] == '1')
      return false; // achou linha → não é GAP
  }
  return true; // só tinha '0' → GAP
}

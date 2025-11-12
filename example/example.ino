#include <sensor_csr.h>

#define NUM_SENSORES 8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {A0,A1,A2,A3,A4,A5,A6,A7};

// Defina seus limiares conforme o teste da pista:
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

void setup() {
  Serial.begin(115200);
  Serial.println("=== Teste Biblioteca sensor_csr ===");
  Serial.println("Calibrando sensores (~4s)...");
  
  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);
  qtr.calibrate(400, false);
  
  Serial.println("Calibração concluída!");
}

void loop() {
  static char bits[NUM_SENSORES+1];
  static int erro = 0;

  const int lastSteps = (erro==0)?0:(erro/1000);
  erro = qtr.ErroSensor(bits, LIMIAR_BRANCO, LIMIAR_PRETO, false, lastSteps);

  Serial.print("bits="); Serial.print(bits);
  Serial.print(" | erro="); Serial.println(erro);
}
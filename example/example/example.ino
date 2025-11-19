/*===============================================================================
  Projeto:    Example Teste Sensor Generico QTRA
  Autor:      César Augusto Victor
===============================================================================*/
#include <sensor_csr.h>

/* ---------------- SENSOR_CSR ---------------- */
#define EMITTER_PIN 11
#define NUM_SENSORES            8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {
  A0, A1, A2, A3, A4, A5, A6, A7
};

/* ---------------- LIMIARES ---------------- */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

int erro = 0;

void setup() {
  Serial.begin(115200);

  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);

  Serial.println("=== Calibrando sensores ===");
  qtr.calibrate(2000, false);
  Serial.println("Calibração concluída!");
}

void loop() {

  //Recebe da biliteoca o erro atual do sensor
  erro = qtr.ErroSensor(LIMIAR_BRANCO, LIMIAR_PRETO);

  //Recebe da função GAP
  bool gap = qtr.gapDetection();

  Serial.print(" erro=");   Serial.print(erro);
  Serial.println(" | gap=");    Serial.print(gap);

}

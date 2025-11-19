/*===============================================================================
  Projeto:    Seguidor de Linha (DRV8833) – Frente = LOW + PWM
  Controle:   PID simples + GAP (usa último erro válido)
  Autor:      César Augusto Victor
===============================================================================*/

#include <sensor_csr.h>
// ------------- PINOUT DRIVE ---------------------*/
#define AIN1 4      // DIR ESQ
#define AIN2 5      // PWM ESQ
#define BIN1 7      // DIR DIR
#define BIN2 9      // PWM DIR
#define DRIVE 6     // STBY

/* ---------------- SENSOR_CSR ---------------- */
#define EMITTER_PIN 11
#define NUM_SENSORES            8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {
  A0, A1, A2, A3, A4, A5, A6, A7
};

/* --------- OUTROS COMPONENTES ------------- */
#define LED      12

/* ---------------- LIMIARES ---------------- */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

/* ---------------- CONTROLE ---------------- */
#define SENSOR_DEBUG 0 // Para encontrar o limiar Ative para 1
#define TURN_SIGN (-1) // Mude para +1 se o robô virar para o lado errado

int   PWM_MAX  = 250;
int   PWM_BASE = 250;

float KP = 0.10;
float KD = 0.80;

int erro = 0;
int erroAnt = 0;

int ultimoErroValido = 0;
int gapCount = 0;

void motorEsquerdaFrente(int pwm);
void motorEsquerdaFreio();
void motorDireitaFrente(int pwm);
void motorDireitaFreio();

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(DRIVE, OUTPUT);
  digitalWrite(DRIVE, HIGH);

  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);

  pinMode(LED, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);

  Serial.println("=== Calibrando sensores ===");
  digitalWrite(LED, HIGH);
  qtr.calibrate(3000, false);
  digitalWrite(LED, LOW);
  Serial.println("Calibração concluída!");
}

void loop() {

  //Recebe da biliteoca o erro atual do sensor
  erro = qtr.ErroSensor(LIMIAR_BRANCO, LIMIAR_PRETO);

#if SENSOR_DEBUG
  Serial.print("bits="); Serial.print(qtr.getBits());
  Serial.print(" | erro="); Serial.println(erro);
  delay(50);
  return;
#endif

  // --------- DETECA O GAP ------------- //
  //Recebe da função GAP
  bool gap = qtr.gapDetection();

  if (!gap) {
    //Seguindo a linha
    ultimoErroValido = erro;
    gapCount = 0;
  } else {
    // Passando por um (GAP) → mantém erro anterior
    gapCount++;
  }

  int erroControle = erro;
  if (gap) {
    erroControle = ultimoErroValido;
  }

  // --------- CONTORLE PID --------- //
  int e = erroControle;
  int d = e - erroAnt;
  erroAnt = e;

  float u = TURN_SIGN * (KP * e + KD * d);

  int pwmL = PWM_BASE + (int)u;
  int pwmR = PWM_BASE - (int)u;

  if (pwmL < 0) pwmL = 0;
  if (pwmL > PWM_MAX) pwmL = PWM_MAX;
  if (pwmR < 0) pwmR = 0;
  if (pwmR > PWM_MAX) pwmR = PWM_MAX;

  /* --------- AÇÕES DO CONTROLE DO ROBÔ --------- */
  if (e >= 5000) { // Linha extrema a direita            
    motorEsquerdaFreio();
    motorDireitaFrente(PWM_MAX);
  }
  else if (e <= -5000) { // Linha extrema a esquerda      
    motorDireitaFreio();
    motorEsquerdaFrente(PWM_MAX);
  }
  else { //Robo segue com o controle PID     
    motorEsquerdaFrente(pwmL); 
    motorDireitaFrente(pwmR);
  }

  /* --------- DEBUG --------- */
  Serial.print("bits=");      Serial.print(qtr.getBits());
  Serial.print(" | erro=");   Serial.print(erroControle);
  Serial.print(" | gap=");    Serial.print(gap);
  Serial.print(" | gapCnt="); Serial.print(gapCount);
  Serial.print(" | pwmL=");   Serial.print(pwmL);
  Serial.print(" | pwmR=");   Serial.println(pwmR);

  delay(5);
}

//----- CONTROLE DE DIREÇÃO DOS MOTORES -----//
void motorEsquerdaFrente(int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;
  digitalWrite(AIN1, LOW); 
  analogWrite(AIN2, pwm);
}

void motorEsquerdaFreio() {
  digitalWrite(AIN1, HIGH);   
  digitalWrite(AIN2, HIGH);
}
void motorDireitaFrente(int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, pwm);
}

void motorDireitaFreio() {
  digitalWrite(BIN1, HIGH); 
  digitalWrite(BIN2, HIGH);
}

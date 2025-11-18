/*
===============================================================================
  Projeto:    Seguidor de Linha (DRV8833) – Frente = LOW + PWM
  Controle:   PID simples + GAP (usa último erro válido)
  Autor:      César Augusto Victor
===============================================================================
*/

#include <sensor_csr.h>

/* ---------------- PINOS DRV8833 ---------------- */
#define AIN1 4      // DIR ESQ
#define AIN2 5      // PWM ESQ
#define BIN1 7      // DIR DIR
#define BIN2 9      // PWM DIR
#define DRIVE 6     // STBY / ENABLE do DRV8833

#define EMITTER_PIN 11
#define LEDPIN      12

/* ---------------- SENSOR_CSR ---------------- */
#define NUM_SENSORES            8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {
  A0, A1, A2, A3, A4, A5, A6, A7
};

/* ---------------- LIMIARES ---------------- */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

/* ---------------- CONTROLE ---------------- */
#define SENSOR_DEBUG 0
#define TURN_SIGN (-1)  // -1 caso esteja virando ao contrário

int   PWM_MAX  = 250;
int   PWM_BASE = 250;

float KP = 0.10;
float KD = 0.80;

int erro = 0;
int erroAnt = 0;

/* GAP */
int ultimoErroValido = 0;
int gapCount = 0;

/* ===================== PROTÓTIPOS ===================== */
void motorEsquerdaFrente(int pwm);
void motorEsquerdaLivre();
void motorEsquerdaFreio();
void motorDireitaFrente(int pwm);
void motorDireitaLivre();
void motorDireitaFreio();

/* ===================== SETUP ===================== */
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

  pinMode(LEDPIN, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);

  Serial.println("=== Calibrando sensores ===");
  digitalWrite(LEDPIN, HIGH);
  qtr.calibrate(3000, false);
  digitalWrite(LEDPIN, LOW);
  Serial.println("Calibração concluída!");
}

/* ===================== LOOP ===================== */
void loop() {

  static char bits[NUM_SENSORES + 1];

  erro = qtr.ErroSensor(bits, LIMIAR_BRANCO, LIMIAR_PRETO, false, 0);

#if SENSOR_DEBUG
  Serial.print("bits="); Serial.print(bits);
  Serial.print(" | erro="); Serial.println(erro);
  delay(50);
  return;
#endif

  /* --------- Detecta GAP --------- */
  bool gap = true;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (bits[i] == '1') { gap = false; break; }
  }

  if (!gap) {
    ultimoErroValido = erro;
    gapCount = 0;
  } else {
    gapCount++;
  }

  int erroControle = erro;
  if (gap) erroControle = ultimoErroValido;

  /* --------- PID --------- */
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

  /* --------- AÇÕES NOS MOTORES --------- */
  // Erro MUITO alto = freio em uma roda para puxar forte
  if (e >= 5000) {             // linha totalmente à direita
    motorEsquerdaFreio();
    motorDireitaFrente(PWM_MAX);
  }
  else if (e <= -5000) {       // linha totalmente à esquerda
    motorDireitaFreio();
    motorEsquerdaFrente(PWM_MAX);
  }
  else {
    // Controle proporcional normal
    motorEsquerdaFrente(pwmL);
    motorDireitaFrente(pwmR);
  }

  /* --------- DEBUG --------- */
  Serial.print("bits=");    Serial.print(bits);
  Serial.print(" | erro="); Serial.print(erroControle);
  Serial.print(" | gap=");  Serial.print(gap);
  Serial.print(" | pwmL="); Serial.print(pwmL);
  Serial.print(" | pwmR="); Serial.println(pwmR);

  delay(5);
}
// --------- ESQUERDA ---------
void motorEsquerdaFrente(int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;

  digitalWrite(DRIVE, HIGH);
  digitalWrite(AIN1, LOW);
  analogWrite(AIN2, pwm);
}

void motorEsquerdaFreio() {
  digitalWrite(DRIVE, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
}

// --------- DIREITA ---------
void motorDireitaFrente(int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;

  digitalWrite(DRIVE, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, pwm);
}

void motorDireitaFreio() {
  digitalWrite(DRIVE, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

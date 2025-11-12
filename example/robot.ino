/*
================================================================================
  Projeto:    Controle Seguidor de Linha com sensor_csr
  Autor:      César Augusto Victor
  Instituição: Universidade Federal do Ceará (UFC - Campus Sobral)
  Curso:      Mestrado em Engenharia Elétrica e de Computação
  Contato:    cesartri2012@gmail.com
  Licença:    MIT License
  Descrição:  Controle PD para robô seguidor de linha utilizando a biblioteca
              sensor_csr (sensores QTR genéricos), com acionamento via DRV8833.
================================================================================
*/

#include <sensor_csr.h>

/* ------------------ PINAGEM DRV8833 (Arduino NANO) ------------------
   Cada lado do driver (A e B) controla um motor:
   - xIN1 e xIN2: controlam a direção
   - Um dos pinos deve ser PWM (velocidade)
   ----------------------------------------------------------
   Tabela de controle (para cada canal):
   | xIN1 | xIN2 | Estado  |
   |------|------|----------|
   | LOW  | PWM  | Frente   |
   | HIGH | HIGH | Freio    |
   | LOW  | LOW  | Solto    |
   ---------------------------------------------------------- */
#define AIN1 4   // digital (ESQUERDO xIN1)
#define AIN2 5   // PWM     (ESQUERDO xIN2)
#define BIN1 7   // digital (DIREITO xIN1)
#define BIN2 9   // PWM     (DIREITO xIN2)
#define SLP_PIN 6
#define LEDPIN 11

/* ------------------ SENSOR_CSR ------------------ */
#define NUM_SENSORES 8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {A0, A1, A2, A3, A4, A5, A6, A7};

/* ------------------ LIMIARES (definidos aqui no .ino) ------------------
   Ajuste conforme os níveis medidos na sua pista:
   - abaixo de LIMIAR_BRANCO = branco (0)
   - acima  de LIMIAR_PRETO  = preto  (1) */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

/* ------------------ CONTROLE DE TAXA ------------------ */
const uint16_t CONTROL_DT_MS = 2;   // ~500 Hz de atualização do controle

/* ------------------ PWM / SUAVIZAÇÃO ------------------
   PWM_MAX: teto de velocidade.
   PASSO_MAX_PWM: quanto o PWM pode variar por ciclo.
   - 0  => sem suavização (resposta imediata)
   - 10 => varia no máx ±10 por ciclo (mais suave) */
int PWM_MAX        = 250;
int PASSO_MAX_PWM  = 0;     // mude para 6..15 se quiser suavizar
int lastL = 0, lastR = 0;   // últimos PWM aplicados (mostrados no Serial)

/* ------------------ CONTROLADOR PD (sem I) ------------------ */
float KP = 0.20f;           // ganho proporcional
float KD = 0.70f;           // ganho derivativo
const int   D_CLAMP = 2000; // limite para derivada (evita pico por ruído)
const float BOOST   = 0.6f; // reforço dinâmico conforme |erro|

int erro = 0, erroAnt = 0;  // erro atual e anterior
unsigned long lastCtrlMs = 0;

/* ------------------ LIMITAÇÃO PASSO A PASSO (OPCIONAL)
   Deixa o PWM "andar" até o alvo devagar, no máximo PASSO_MAX_PWM por ciclo.
   - Se PASSO_MAX_PWM == 0, não altera nada (retorna o alvo).
---------------------------------------------------------------- */
static inline int limitaPassoPWM(int alvo, int atual, int passoMax) {
  if (passoMax <= 0) return alvo;   // suavização desativada
  int delta = alvo - atual;
  if (delta >  passoMax) delta =  passoMax;
  if (delta < -passoMax) delta = -passoMax;
  return atual + delta;
}

/* ------------------ FUNÇÕES MOTOR (DRV8833) ------------------ */
void motorLeftForward(uint8_t pwm) {
  // Frente → AIN1=LOW, AIN2=PWM
  digitalWrite(AIN1, LOW);
  analogWrite(AIN2, pwm);
}
void motorLeftBrake() {
  // Freio (curto ativo) → AIN1=HIGH, AIN2=HIGH
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
}
void motorLeftCoast() {
  // Livre (coast) → AIN1=LOW, AIN2=LOW
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void motorRightForward(uint8_t pwm) {
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, pwm);
}
void motorRightBrake() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}
void motorRightCoast() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

/* ------------------ SAÍDA PRINCIPAL (controle dos motores)
   - Limita para 0..PWM_MAX.
   - Aplica suavização passo-a-passo (se PASSO_MAX_PWM > 0).
   - Envia comandos ao DRV8833: frente, freio ou livre.
---------------------------------------------------------------- */
void outputDrive(uint16_t pwmL, bool brakeL, uint16_t pwmR, bool brakeR) {
  // 1) Limita PWM em 0..PWM_MAX
  if ((int)pwmL < 0) pwmL = 0;
  if ((int)pwmR < 0) pwmR = 0;
  if (pwmL > (uint16_t)PWM_MAX) pwmL = PWM_MAX;
  if (pwmR > (uint16_t)PWM_MAX) pwmR = PWM_MAX;

  // 2) Suavização opcional (mais fácil de entender que applySlew)
  pwmL = limitaPassoPWM(pwmL, lastL, PASSO_MAX_PWM);
  pwmR = limitaPassoPWM(pwmR, lastR, PASSO_MAX_PWM);

  // 3) Habilita o driver e LED (ativos em HIGH)
  digitalWrite(SLP_PIN, HIGH);
  digitalWrite(LEDPIN, HIGH);

  // 4) Aplica comandos de cada lado
  if (brakeL) motorLeftBrake();
  else if (pwmL > 0) motorLeftForward(pwmL);
  else motorLeftCoast();

  if (brakeR) motorRightBrake();
  else if (pwmR > 0) motorRightForward(pwmR);
  else motorRightCoast();

  // 5) Guarda os últimos PWMs (debug/telemetria)
  lastL = pwmL;
  lastR = pwmR;
}

/* ------------------ SAÍDA IMEDIATA (sem suavização)
   Usada como “rede de segurança” quando perde a linha:
   um lado em freio, o outro no máximo, para girar rápido.
---------------------------------------------------------------- */
void outputDriveImmediate(uint16_t pwmL, bool brakeL, uint16_t pwmR, bool brakeR) {
  // Limite 0..PWM_MAX
  if ((int)pwmL < 0) pwmL = 0;
  if ((int)pwmR < 0) pwmR = 0;
  if (pwmL > (uint16_t)PWM_MAX) pwmL = PWM_MAX;
  if (pwmR > (uint16_t)PWM_MAX) pwmR = PWM_MAX;

  digitalWrite(SLP_PIN, HIGH);
  digitalWrite(LEDPIN, HIGH);

  if (brakeL) motorLeftBrake();
  else if (pwmL > 0) motorLeftForward(pwmL);
  else motorLeftCoast();

  if (brakeR) motorRightBrake();
  else if (pwmR > 0) motorRightForward(pwmR);
  else motorRightCoast();

  // Atualiza debug
  lastL = pwmL;
  lastR = pwmR;
}

/* ------------------ SETUP ------------------ */
void setup() {
  Serial.begin(115200);

  pinMode(LEDPIN, OUTPUT);
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);   // LED ON para indicar sistema ativo
  digitalWrite(SLP_PIN, HIGH);  // habilita o DRV8833

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Inicializa e calibra sensores (mapeia 0..1000 por canal)
  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);

  Serial.println("=== Calibrando sensores (~4s) ===");
  qtr.calibrate(400, false);
  Serial.println("Calibração concluída!\n");
}

/* ------------------ LOOP PRINCIPAL ------------------
   1) Lê erro a partir dos sensores (via sensor_csr)
   2) Aplica controle PD (com BOOST)
   3) Comanda os motores
---------------------------------------------------------------- */
void loop() {
  static char bits[NUM_SENSORES+1];

  unsigned long now = millis();
  if (now - lastCtrlMs >= CONTROL_DT_MS) {
    lastCtrlMs = now;

    /* 1) Leitura de sensores e erro discreto (-4000..+4000)
       - bits[] recebe o padrão '0'/'1' por sensor (para debug)
       - lastSteps ajuda a decidir o lado quando a linha some (memória) */
    const int lastSteps = (erro == 0) ? 0 : (erro/1000);
    erro = qtr.ErroSensor(bits, LIMIAR_BRANCO, LIMIAR_PRETO, /*useAmbient=*/false, lastSteps);

    int pwmL = 0, pwmR = 0;

    /* 2) Casos extremos: linha perdida → giro forte com freio do lado interno */
    if (erro <= -4000) {             // linha muito à ESQUERDA
      pwmL = PWM_MAX; pwmR = 0;       // gira para a DIREITA
      outputDriveImmediate(pwmL, false, pwmR, true);
    }
    else if (erro >= +4000) {        // linha muito à DIREITA
      pwmL = 0; pwmR = PWM_MAX;       // gira para a ESQUERDA
      outputDriveImmediate(pwmL, true, pwmR, false);
    }

    /* 3) Controle PD contínuo (curvas suaves/fechadas com BOOST) */
    else {
      int e = erro;           // erro atual
      int d = e - erroAnt;    // derivada (variação do erro)
      erroAnt = e;

      // Limita a derivada para não reagir a ruído
      if (d >  D_CLAMP) d =  D_CLAMP;
      if (d < -D_CLAMP) d = -D_CLAMP;

      // BOOST: aumenta o ganho conforme |erro| (ajuda a "fechar" curvas)
      float scale = 1.0f + BOOST * (float)(abs(e)) / 4000.0f;

      // Sinal de controle (negativo corrige para o lado certo)
      float u = -(KP*e + KD*d) * scale;

      // Limites tradicionais em 'adj'
      int adj = (int)u;
      if (adj >  PWM_MAX) adj =  PWM_MAX;
      if (adj < -PWM_MAX) adj = -PWM_MAX;

      // Redistribuição simétrica em torno de PWM_MAX
      // adj>0 → vira à ESQUERDA (aumenta E, diminui D)
      // adj<0 → vira à DIREITA  (diminui E, aumenta D)
      pwmL = PWM_MAX + (adj / 2);
      pwmR = PWM_MAX - (adj / 2);

      // Garante 0..PWM_MAX
      if (pwmL > PWM_MAX) pwmL = PWM_MAX;
      if (pwmL < 0)       pwmL = 0;
      if (pwmR > PWM_MAX) pwmR = PWM_MAX;
      if (pwmR < 0)       pwmR = 0;

      outputDrive(pwmL, false, pwmR, false);
    }

    /* 4) Telemetria para ajuste fino (Serial Plotter / Monitor) */
    Serial.print("SENSORES= "); Serial.print(bits);
    Serial.print(" | ERRO=");   Serial.print(erro);
    Serial.print(" | PWME=");   Serial.print(lastL);
    Serial.print(" | PWMD=");   Serial.println(lastR);
  }
}

/*
================================================================================
  Projeto:    Seguidor de Linha (DRV8833) com GAP dinâmico e resposta rápida
  Autor:      César Augusto Victor
  Instituição: Universidade Federal do Ceará (UFC - Sobral)
  Curso:      Mestrado em Engenharia Elétrica e de Computação
  Contato:    cesartri2012@gmail.com
  Licença:    MIT License
================================================================================
*/

#include <sensor_csr.h>

/* ===================== AJUSTES RÁPIDOS ===================== */
// Se virar para o lado errado, troque +1 ↔ -1
#define TURN_SIGN (+1)      // +1 ou -1 (inverte sentido do controle)
// Se motores estiverem cruzados (E↔D), use 1
#define SWAP_LR   (0)       // 0 = normal, 1 = troca ESQ↔DIR
// Modo diagnóstico: imprime sensores/bits/erro e NÃO dirige
#define SENSOR_DEBUG (0)
// Reduz a taxa de impressão para não bloquear o loop
#define SERIAL_EVERY_MS (25)
/* =========================================================== */

/* ------------------ PINAGEM DRV8833 (Arduino NANO) ------------------ */
#define AIN1 4   // digital  (motor ESQ xIN1)
#define AIN2 5   // PWM      (motor ESQ xIN2)
#define BIN1 7   // digital  (motor DIR xIN1)
#define BIN2 9   // PWM      (motor DIR xIN2)
#define SLP_PIN 6
#define LEDPIN 11

/* ------------------ SENSOR_CSR ------------------ */
#define NUM_SENSORES 8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {A0, A1, A2, A3, A4, A5, A6, A7};

/* ------------------ LIMIARES (ajuste conforme pista) ------------------ */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

/* ------------------ CONTROLE ------------------ */
const uint16_t CONTROL_DT_MS = 1;    // ~1 kHz (muito responsivo)
int PWM_MAX = 220;

float KP = 0.22f;                    // P para fechar curva
float KD = 0.60f;                    // D para suavizar
const int   D_CLAMP = 2000;          // limita derivada
const float BOOST   = 0.50f;         // ganho cresce com |erro|

int erro = 0, erroAnt = 0;
int lastNonZeroErro = 0;             // memória do último erro válido
unsigned long lastCtrlMs = 0;
unsigned long lastPrintMs = 0;

/* ------------------ GAP dinâmico (super curto) ------------------
   - bits == "00000000"
     • 1º ciclo: seguir reto (70–85% PWM_MAX)
     • 2º ciclo em diante: já aplica viés para o lado do lastNonZeroErro
   - Se erroAnt for muito grande (±3500): pivot com freio (rede de segurança)
---------------------------------------------------------------- */
const int GAP_KEEP_STEPS = 3500;
int gapTicks = 0;                   // quantos ciclos seguidos em 00000000

/* ------------------ Motores: primitivas ------------------ */
static inline void motorLeftForward(uint8_t pwm){
  digitalWrite(AIN1, LOW);  analogWrite(AIN2, pwm);
}
static inline void motorLeftBrake(){
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
}
static inline void motorLeftCoast(){
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
}
static inline void motorRightForward(uint8_t pwm){
  digitalWrite(BIN1, LOW);  analogWrite(BIN2, pwm);
}
static inline void motorRightBrake(){
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
}
static inline void motorRightCoast(){
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
}

/* Saída com opção de SWAP_LR */
void driveForwardLR(int pwmL, int pwmR){
  if (pwmL < 0) pwmL = 0; if (pwmL > PWM_MAX) pwmL = PWM_MAX;
  if (pwmR < 0) pwmR = 0; if (pwmR > PWM_MAX) pwmR = PWM_MAX;
  digitalWrite(SLP_PIN, HIGH);
  if (!SWAP_LR) {
    motorLeftForward((uint8_t)pwmL);
    motorRightForward((uint8_t)pwmR);
  } else {
    motorRightForward((uint8_t)pwmL);
    motorLeftForward((uint8_t)pwmR);
  }
}
void pivotBrakeRightMax(){ // gira para a DIREITA (trava roda DIR, ESQ avança)
  digitalWrite(SLP_PIN, HIGH);
  if (!SWAP_LR){ motorLeftForward(PWM_MAX); motorRightBrake(); }
  else         { motorRightForward(PWM_MAX); motorLeftBrake(); }
}
void pivotBrakeLeftMax(){  // gira para a ESQUERDA (trava roda ESQ, DIR avança)
  digitalWrite(SLP_PIN, HIGH);
  if (!SWAP_LR){ motorLeftBrake();           motorRightForward(PWM_MAX); }
  else         { motorRightBrake();          motorLeftForward(PWM_MAX);  }
}

/* ------------------ Setup ------------------ */
void setup(){
  Serial.begin(115200);

  pinMode(LEDPIN, OUTPUT);
  pinMode(SLP_PIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(SLP_PIN, HIGH);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(PinosSensores, NUM_SENSORES);
  qtr.setSamplesPerSensor(NUM_AMOSTRAS_POR_SENSOR);

  Serial.println("=== Calibrando sensores (~4s) ===");
  qtr.calibrate(400, false);
  Serial.println("Calibração concluída!\n");
}

/* ------------------ Loop ------------------ */
void loop(){
  static char bits[NUM_SENSORES+1];

  unsigned long now = millis();
  if (now - lastCtrlMs >= CONTROL_DT_MS){
    lastCtrlMs = now;

    // Lê erro e bits (com limiares definidos aqui no .ino)
    const int lastSteps = (erro == 0) ? 0 : (erro / 1000);
    erro = qtr.ErroSensor(bits, LIMIAR_BRANCO, LIMIAR_PRETO, /*ambient*/false, lastSteps);

#if SENSOR_DEBUG
    Serial.print("bits="); Serial.print(bits);
    Serial.print(" | erro="); Serial.println(erro);
    return;
#endif

    // Detecta se todos os sensores viram branco (gap)
    bool todosZero = true;
    for (int i = 0; i < NUM_SENSORES; i++){
      if (bits[i] == '1'){ todosZero = false; break; }
    }

    // Atualiza memória quando NÃO está tudo zero
    if (!todosZero) {
      lastNonZeroErro = erro;
      gapTicks = 0;
    }

    if (todosZero) {
      gapTicks++;

      // Se erro anterior for muito grande, prioriza retorno agressivo
      if (erroAnt >= GAP_KEEP_STEPS) {
        pivotBrakeLeftMax();    // linha estava à direita → gira à esquerda
      }
      else if (erroAnt <= -GAP_KEEP_STEPS) {
        pivotBrakeRightMax();   // linha estava à esquerda → gira à direita
      }
      else {
        // erro anterior pequeno: atravessar gap rápido
        if (gapTicks == 1) {
          // Só 1 ciclo reto, PWM reduzido (para não “passar do ponto”)
          int v = (int)(PWM_MAX * 0.75f);
          driveForwardLR(v, v);
        } else {
          // A partir do 2º ciclo, já aplica viés pelo último erro conhecido
          int sign = (lastNonZeroErro >= 0) ? +1 : -1;
          int base = (int)(PWM_MAX * 0.70f);
          int bias = (int)(PWM_MAX * 0.30f);

          // sign >0: vira levemente à ESQUERDA | sign <0: à DIREITA
          int pwmL = base + sign * bias;
          int pwmR = base - sign * bias;
          driveForwardLR(pwmL, pwmR);
        }
      }
      // Não atualiza erroAnt em gap: preserva memória de direção
    }
    else {
      // ===== CONTROLE NORMAL =====
      int e = erro;
      int d = e - erroAnt;
      erroAnt = e;  // atualiza memória do erro

      // limita derivada (anti-ruído)
      if (d >  D_CLAMP) d =  D_CLAMP;
      if (d < -D_CLAMP) d = -D_CLAMP;

      // ganho dinâmico leve
      float scale = 1.0f + BOOST * (float)(abs(e)) / 4000.0f;

      // Se virar para o lado errado, troque TURN_SIGN
      float u = TURN_SIGN * (-(KP * e + KD * d) * scale);

      int adj = (int)u;
      if (adj >  PWM_MAX) adj =  PWM_MAX;
      if (adj < -PWM_MAX) adj = -PWM_MAX;

      // adj > 0 → vira ESQ | adj < 0 → vira DIR
      int pwmL = PWM_MAX + (adj / 2);
      int pwmR = PWM_MAX - (adj / 2);

      if (pwmL > PWM_MAX) pwmL = PWM_MAX; if (pwmL < 0) pwmL = 0;
      if (pwmR > PWM_MAX) pwmR = PWM_MAX; if (pwmR < 0) pwmR = 0;

      // Rede de segurança extrema (pivot com freio)
      if (erro <= -4000)      pivotBrakeRightMax();
      else if (erro >= 4000)  pivotBrakeLeftMax();
      else                    driveForwardLR(pwmL, pwmR);
    }

    // Debug não-bloqueante (evita travar o loop)
    if (now - lastPrintMs >= SERIAL_EVERY_MS){
      lastPrintMs = now;
      Serial.print("bits="); Serial.print(bits);
      Serial.print(" | e=");  Serial.print(erro);
      Serial.print(" | e_last="); Serial.print(erroAnt);
      Serial.print(" | lastNZ="); Serial.print(lastNonZeroErro);
      Serial.print(" | gapTicks="); Serial.print(gapTicks);
      Serial.print(" | PWM_MAX="); Serial.print(PWM_MAX);
      Serial.print(" | sign="); Serial.print((int)TURN_SIGN);
      Serial.print(" | swap="); Serial.println((int)SWAP_LR);
    }
  }
}

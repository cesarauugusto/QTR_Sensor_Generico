/*
================================================================================
  Projeto:    Seguidor de Linha (L298N) com GAP dinâmico e resposta rápida
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
#define TURN_SIGN (+1)          // +1 ou -1 (inverte sentido do controle)
// Se os motores estiverem cruzados (E↔D), use 1
#define SWAP_LR   (0)           // 0 = normal, 1 = troca ESQ↔DIR
// Modo diagnóstico: imprime sensores/bits/erro e NÃO dirige
#define SENSOR_DEBUG (0)
// Reduz a taxa de impressão para não bloquear o loop
#define SERIAL_EVERY_MS (25)
/* =========================================================== */

/* ------------------ PINAGEM L298N (UNO/NANO) ------------------
   Motor Esquerdo: IN1, IN2, ENA(PWM)
   Motor Direito : IN3, IN4, ENB(PWM)
---------------------------------------------------------------- */
#define IN1 4
#define IN2 5
#define ENA 6   // PWM

#define IN3 7
#define IN4 8
#define ENB 9   // PWM

#define LEDPIN 13

/* ------------------ SENSOR_CSR ------------------ */
#define NUM_SENSORES 8
#define NUM_AMOSTRAS_POR_SENSOR 5
sensor_csr qtr;
const uint8_t PinosSensores[NUM_SENSORES] = {A0, A1, A2, A3, A4, A5, A6, A7};

/* ------------------ LIMIARES (ajuste conforme pista) ------------------
   Verifique com SENSOR_DEBUG=1:
   - abaixo de LIMIAR_BRANCO => branco (0)
   - acima  de LIMIAR_PRETO  => preto  (1) */
#define LIMIAR_BRANCO 600
#define LIMIAR_PRETO  900

/* ------------------ CONTROLE ------------------ */
const uint16_t CONTROL_DT_MS = 1;   // ~1 kHz (muito responsivo)
int PWM_MAX = 220;                  // teto de PWM (0..255)

float KP = 0.22f;                   // P para “fechar” curva
float KD = 0.60f;                   // D para suavizar
const int   D_CLAMP = 2000;         // limita derivada (anti-ruído)
const float BOOST   = 0.50f;        // ganho cresce com |erro|

int erro = 0, erroAnt = 0;
int lastNonZeroErro = 0;            // memória do último erro válido
unsigned long lastCtrlMs  = 0;
unsigned long lastPrintMs = 0;

/* ------------------ GAP dinâmico (super curto) ------------------
   - bits == "00000000"
     • 1º ciclo: seguir reto (70–85% PWM_MAX)
     • 2º ciclo em diante: aplica viés usando lastNonZeroErro
   - Se erroAnt for muito grande (±3500): “pivot” com freio (agressivo)
---------------------------------------------------------------- */
const int GAP_KEEP_STEPS = 3500;
int gapTicks = 0; // quantos ciclos seguidos em 00000000

/* ------------------ PRIMITIVAS DE MOTOR (L298N) ------------------
   Esquerdo:
     Frente: IN1=HIGH, IN2=LOW,  ENA=pwm
     Freio : IN1=HIGH, IN2=HIGH, ENA=0
     Livre : IN1=LOW,  IN2=LOW,  ENA=0
   Direito:
     Frente: IN3=HIGH, IN4=LOW,  ENB=pwm
     Freio : IN3=HIGH, IN4=HIGH, ENB=0
     Livre : IN3=LOW,  IN4=LOW,  ENB=0
------------------------------------------------------------------ */
static inline void motorLeftForward(uint8_t pwm){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwm);
}
static inline void motorLeftBrake(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 0);
}
static inline void motorLeftCoast(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

static inline void motorRightForward(uint8_t pwm){
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwm);
}
static inline void motorRightBrake(){
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 0);
}
static inline void motorRightCoast(){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

/* Camada de saída com opção de SWAP_LR */
void driveForwardLR(int pwmL, int pwmR){
  if (pwmL < 0) pwmL = 0; if (pwmL > PWM_MAX) pwmL = PWM_MAX;
  if (pwmR < 0) pwmR = 0; if (pwmR > PWM_MAX) pwmR = PWM_MAX;

  if (!SWAP_LR){
    motorLeftForward((uint8_t)pwmL);
    motorRightForward((uint8_t)pwmR);
  } else {
    motorRightForward((uint8_t)pwmL);
    motorLeftForward((uint8_t)pwmR);
  }
}
void pivotBrakeRightMax(){ // gira para a DIREITA (trava roda DIR, ESQ avança)
  if (!SWAP_LR){ motorLeftForward(PWM_MAX); motorRightBrake(); }
  else         { motorRightForward(PWM_MAX); motorLeftBrake(); }
}
void pivotBrakeLeftMax(){  // gira para a ESQUERDA (trava roda ESQ, DIR avança)
  if (!SWAP_LR){ motorLeftBrake();           motorRightForward(PWM_MAX); }
  else         { motorRightBrake();          motorLeftForward(PWM_MAX);  }
}

/* ------------------ Setup ------------------ */
void setup(){
  Serial.begin(115200);

  pinMode(LEDPIN, OUTPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  digitalWrite(LEDPIN, HIGH);

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
    return; // não move motores no modo debug
#endif

    // Checa GAP (todos os sensores 0)
    bool todosZero = true;
    for (int i = 0; i < NUM_SENSORES; i++){
      if (bits[i] == '1'){ todosZero = false; break; }
    }

    // Atualiza memória quando NÃO está em gap
    if (!todosZero){
      lastNonZeroErro = erro;
      gapTicks = 0;
    }

    if (todosZero){
      gapTicks++;

      // Se erro anterior era muito grande, prioriza retorno agressivo
      if (erroAnt >= GAP_KEEP_STEPS){
        pivotBrakeLeftMax();   // linha estava à direita → girar à esquerda
      }
      else if (erroAnt <= -GAP_KEEP_STEPS){
        pivotBrakeRightMax();  // linha estava à esquerda → girar à direita
      }
      else {
        // erroAnt pequeno: atravessa gap de forma curtíssima
        if (gapTicks == 1){
          int v = (int)(PWM_MAX * 0.75f);  // 1 ciclo reto com PWM reduzido
          driveForwardLR(v, v);
        } else {
          // do 2º ciclo em diante, viés usando lastNonZeroErro
          int sign = (lastNonZeroErro >= 0) ? +1 : -1;
          int base = (int)(PWM_MAX * 0.70f);
          int bias = (int)(PWM_MAX * 0.30f);
          int pwmL = base + sign * bias;   // sign>0: vira levemente à ESQ
          int pwmR = base - sign * bias;   // sign<0: vira levemente à DIR
          driveForwardLR(pwmL, pwmR);
        }
      }
      // mantém erroAnt (memória) em gap
    }
    else {
      // ===== CONTROLE NORMAL =====
      int e = erro;
      int d = e - erroAnt;
      erroAnt = e; // atualiza memória

      // limita derivada (anti-ruído)
      if (d >  D_CLAMP) d =  D_CLAMP;
      if (d < -D_CLAMP) d = -D_CLAMP;

      // ganho dinâmico leve
      float scale = 1.0f + BOOST * (float)(abs(e)) / 4000.0f;

      // se virar para o lado errado, mude TURN_SIGN
      float u = TURN_SIGN * (-(KP * e + KD * d) * scale);

      int adj = (int)u;
      if (adj >  PWM_MAX) adj =  PWM_MAX;
      if (adj < -PWM_MAX) adj = -PWM_MAX;

      // adj > 0 → vira ESQ (mais E, menos D) | adj < 0 → vira DIR
      int pwmL = PWM_MAX + (adj / 2);
      int pwmR = PWM_MAX - (adj / 2);
      if (pwmL > PWM_MAX) pwmL = PWM_MAX; if (pwmL < 0) pwmL = 0;
      if (pwmR > PWM_MAX) pwmR = PWM_MAX; if (pwmR < 0) pwmR = 0;

      // Rede de segurança extrema (pivot com freio)
      if (erro <= -4000)      pivotBrakeRightMax();
      else if (erro >= 4000)  pivotBrakeLeftMax();
      else                    driveForwardLR(pwmL, pwmR);
    }

    // Debug não-bloqueante
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

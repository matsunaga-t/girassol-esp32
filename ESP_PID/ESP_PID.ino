#include <arduino.h>
#include <cmath>
#include <ESP32Servo.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <HTTPClient.h>
#include <WiFi.h>

// ======================= DEFINIÇÃO DOS PINOS  ==============
// ============ NÃO MUDAR SEM MOTIVO ========================
// Pinos dos LDRs
#define LEFT_LDR_PIN       36     /* Pino do LDR esquerdo */
#define RIGHT_LDR_PIN      39     /* Pino do LDR direito */

#define LEFT_LDR2_PIN      34     /* Pino do LDR esquerdo secundário */
#define RIGHT_LDR2_PIN     35     /* Pino do LDR direito secundário */

// Pinos dos servos
#define MOTOR_OUT          25     // Pino da saída para o servomotor

// Pinos de entrada
#define INPUT1_PIN         32
#define INPUT2_PIN         33
#define INPUT3_PIN         35
#define HALT_PIN           26

// ======================= CONFIGURAÇÕES ADICIONAIS =================
// LDR esquerdo
#define LEFT_LDR           2      
// LDR direito
#define RIGHT_LDR          4      

#define USE_SECONDARY_LDR  0      // 1 para usar os LDRs secundários

// LDR esquerdo secundário
#define LEFT_LDR2          1      
// LDR direito secundário
#define RIGHT_LDR2         3      

// ----------------------- Configurações do wifi ------------------------
#define USE_WIFI           1      // 1 para enviar dados para o servidor

const char* ssid     = "pi-dashboard";   // rede criada pelo Raspberry Pi (AP)
const char* password = "dashboard123";
const char* serverIP = "http://192.168.4.1:5000/dados";

// ----------------------- Configurações do condicionamento de sinais ----------------
#define MOVING_AVERAGE_SAMPLES 5  // (int) Quantidade de amostras para a média móvel

  // ......................... Valores do normalizador de LDR ....................
  // ................ NÃO MUDAR SEM MOTIVO .......................................
  #define LDR1_ALPHA           1 / -0.615932450836737f    // (float) Parâmetro alpha do LDR1
  #define LDR2_ALPHA           1 / -0.570998772975310f    // (float) Parâmetro alpha do LDR2
  #define LDR3_ALPHA           1 / -0.578058151321358f    // (float) Parâmetro alpha do LDR3
  #define LDR4_ALPHA           1 / -0.552624264297261f    // (float) Parâmetro alpha do LDR4

  #define LDR1_BETA            std::powf((320) / std::expf(11.3373904862202f), LDR1_ALPHA)  // (float) Parâmetro beta do LDR1
  #define LDR2_BETA            std::powf((320) / std::expf(10.9327655047729f), LDR2_ALPHA)  // (float) Parâmetro beta do LDR2
  #define LDR3_BETA            std::powf((320) / std::expf(11.2481036463693f), LDR3_ALPHA)  // (float) Parâmetro beta do LDR3
  #define LDR4_BETA            std::powf((320) / std::expf(10.6852316542604f), LDR4_ALPHA)  // (float) Parâmetro beta do LDR4

// ----------------------- Configurações de controle ----------------------
#define CONTROLL_MOTOR     1      // 1 para controlar o servomotor (enviar saída PWM)

  // ......................... Ajuste do PWM .................................
  #define PWM_FREQUENCY        50         // (int) Frequência do PWM
  #define PWM_RESOLUTION       20         // (int) Resolução do PWM
  #define LOWER_CLAMP          1000       // (int) Limite inferior da saída PWM
  #define UPPER_CLAMP          2000       // (int) Limite superior da saída PWM
  #define PWM_CENTER           1500       // (int) Valor central do PWM 

  // ......................... Ajuste do controlador PID ..............................................
  #define PID_SAMPLE_RATE      10        // (int) Intervalo de tempo, em ms, entre atualizações no PID

  #define P_GAIN               0.120      // (float) Ganho proporcional do PID
  #define I_GAIN               0//.0975   // (float) Ganho integral do PID
  #define D_GAIN               0//.185    // (float) Ganho diferencial do PID

#define CHANGE_PID_GAIN        1          // 1 para mudar os ganhos do PID

  // ........................... Ajuste dos parâmetros do controlador PID ............................
  #define GAIN_TYPE              0             // Ganho a ser modificado. 0 para Kp, 1 para Ki e 2 para Kd
  #define GAIN_MULT              0.5f          // Multiplicador do ganho na entrada

// --------------------------- Configurações do acelerômetro ---------------------------------------
#define USE_ACCELEROMETER      0           // 1 para usar o acelerômetro

  // ............................. Limites do acelerõmetro ................................
  #define X_ACCEL_RANGE            2.0         // Faixa de aceleração do eixo X
  #define X_ACCEL_OFFSET           0.0         // Offset da aceleração do eixo X
      
// --------------------- Configurações de impressão ----------------------------
#define PRINT_INPUT_INFO 1    // Imprime informação da entrada dos potenciometros
#define PRINT_PID_GAINS  1    // Imprime os ganhos do controlador PID
#define PRINT_SYSTEM_IO  1    // Imprime a entrada e saída do sistema

// ==================== MAIS MACROS PARA AJUDAR =====================================
// ==================== NÃO MUDAR SEM MOTIVO ============================
#define LDR_PARAMS_HELPER(ldr_number) LDR##ldr_number##_ALPHA, LDR##ldr_number##_BETA
#define LDR_PARAMS(ldr_number)        LDR_PARAMS_HELPER(ldr_number)

#define XTABLE_LDR \
  /*      pino      numero físisco    entrada crua    classe ma      classe norm        média      iluminância   ilum total*/ \
  X( LEFT_LDR_PIN,     LEFT_LDR,      leftLDR_raw,    leftLDR_ma,    leftLDR_norm,   leftLDR_mean,  leftLDR_E,      leftE) \
  X(RIGHT_LDR_PIN,    RIGHT_LDR,     rightLDR_raw,   rightLDR_ma,   rightLDR_norm,  rightLDR_mean, rightLDR_E,     rightE) \
  XTABLE_LDR_SECONDARY

#if USE_SECONDARY_LDR
  #define XTABLE_LDR_SECONDARY \
    X2( LEFT_LDR2_PIN,  LEFT_LDR2,  leftLDR2_raw,  leftLDR2_ma,  leftLDR2_norm,  leftLDR2_mean,  leftLDR2_E,   leftE) \
    X2(RIGHT_LDR2_PIN, RIGHT_LDR2, rightLDR2_raw, rightLDR2_ma, rightLDR2_norm, rightLDR2_mean, rightLDR2_E,  rightE)
#else
  #define XTABLE_LDR_SECONDARY
#endif

// key: X(LDR_pin, LDR_number, LDR_raw, LDR_ma, LDR_norm, LDR_mean, LDR_E, total_E)

// ============================================================================================

// definições para o programa
#define X2 X
#define X(LDR_pin, LDR_number, LDR_raw, LDR_ma, LDR_norm, LDR_mean, LDR_E, total_E) \
  uint32_t LDR_raw;   \
  float    LDR_E;  \
  /* inicialização das classes */  \
  MovingAverage LDR_ma   = MovingAverage(MOVING_AVERAGE_SAMPLES);  \
  LDRNormalizer LDR_norm = LDRNormalizer(LDR_PARAMS(LDR_number));
XTABLE_LDR
#undef X
#undef X2

float leftE, rightE;                     // Iluminância dos LDRs
long PID_output;                         // saída do PID
Servo motor;
#if CHANGE_PID_GAIN
  float kp, ki, kd;                      // Ganhos do PID
  float input1;
  float input2;
  float input3;
#endif
bool running = false;

#if USE_ACCELEROMETER
  // ajustar
  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif

PIDController pidCon = PIDController(P_GAIN, I_GAIN, D_GAIN, &leftE, &rightE, &PID_output, LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER);


void conectarWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao AP do Pi");
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi conectado! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nFalha no Wi-Fi — continuando sem envio de dados.");
  }
}

void enviarDados() {
  if (WiFi.status() != WL_CONNECTED) return;

  // JSON com todos os dados do sistema
  String payload = "{";
  payload += "\"ldr1_raw\":"          + String(leftLDR_raw / 1000.0f,  4)    + ",";
  payload += "\"ldr2_raw\":"          + String(rightLDR_raw / 1000.0f, 4)    + ",";
  payload += "\"ldr1_mean\":"         + String(leftLDR_ma.getValue(),  4)    + ",";
  payload += "\"ldr2_mean\":"         + String(rightLDR_ma.getValue(), 4)    + ",";
  payload += "\"ldr1_illuminance\":"  + String(leftE,   4)                   + ",";
  payload += "\"ldr2_illuminance\":"  + String(rightE,  4)                   + ",";
  payload += "\"erro\":"              + String(leftE - rightE,  4)           + ",";
  payload += "\"pid_output\":"        + String(PID_output)                   + ",";
  payload += "\"pwm_pct\":"           + String( (PID_output - PWM_CENTER) / (UPPER_CLAMP - LOWER_CLAMP) * 2 * 100 , 2);
  payload += "}";

  HTTPClient http;
  http.begin(serverIP);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);

  if (code != 200) {
    Serial.print("[HTTP] erro: ");
    Serial.println(code);
  }
  http.end();
}

void setup() {
  Serial.begin(9600);

  #if CONTROLL_MOTOR
    motor.attach(MOTOR_OUT, LOWER_CLAMP, UPPER_CLAMP);
    motor.write(PWM_CENTER);
  #endif

  #define X2 X
  #define X(LDR_pin, LDR_number, LDR_raw, LDR_ma, LDR_norm, LDR_mean, LDR_E, total_E) \
    pinMode(LDR_pin, INPUT);
  XTABLE_LDR
  #undef X
  #undef X2

  #if USE_ACCELEROMETER
    if (!accel.begin()) {
      Serial.println("Não foi possível encontrar o ADXL345. Verifique as conexões!");
      while (1);
    }
    accel.setRange(ADXL345_RANGE_2_G);
  #endif

  #if CHANGE_PID_GAIN
    pinMode(INPUT1_PIN, INPUT);
    pinMode(INPUT2_PIN, INPUT);
    pinMode(INPUT3_PIN, INPUT);
    pinMode(HALT_PIN, INPUT_PULLDOWN);
  #endif

  #if USE_WIFI
    conectarWiFi();
  #endif
}

void loop(){
  #if USE_ACCELEROMETER
    sensors_event_t event; 
    accel.getEvent(&event);
  #endif

  running = running ^ digitalRead(HALT_PIN);
  #if CHANGE_PID_GAIN
    input1 = (analogReadMilliVolts(INPUT1_PIN) - 142) / (3165.0 - 142.0);
    input2 = (analogReadMilliVolts(INPUT2_PIN) - 142) / (3165.0 - 142.0);
    input3 = (analogReadMilliVolts(INPUT3_PIN) - 142) / (3165.0 - 142.0);

    pidCon.setAllGains(input1 * GAIN_MULT, input2 * GAIN_MULT, input3 * GAIN_MULT);
  #endif

  // Imprime a entrada do potenciômetro
  #if PRINT_INPUT_INFO
    #if CHANGE_PID_GAIN
      Serial.printf("[%5.3f, %5.3f, %5.3f] ", input1, input2, input3);
    #endif
    Serial.printf("%s  ", running ? "RUNNING" : "HALTED ");
  #endif

  // Ganhos do PID
  #if PRINT_PID_GAINS
    pidCon.getAllGains(&kp, &ki, &kd);
    Serial.printf("PID = {%5.3f, %5.3f, %5.3f}  ", kp, ki, kd);
  #endif

  leftE = rightE = 0.0f;
  #define X2 X
  #define X(LDR_pin, LDR_number, LDR_raw, LDR_ma, LDR_norm, LDR_mean, LDR_E, total_E) \
    LDR_raw = analogReadMilliVolts(LDR_pin);  \
    LDR_ma.update(LDR_raw / 1000.0f);  \
    LDR_E = LDR_norm.toIlluminance(LDR_ma.getValue());  \
    total_E += LDR_E;
  XTABLE_LDR
  #undef X
  #undef X2
  
  PID_output = pidCon.update();
  if(running)
    motor.write(PID_output);
  else
    motor.write(1500);

  #if PRINT_SYSTEM_IO
    Serial.printf("raw = (%4d, %4d)  ma = (%5.2f, %5.2f)  E = (%7.0f, %7.0f)  ln(E) = (%5.2f, %5.2f) -> %d",
      leftLDR_raw, rightLDR_raw,
      leftLDR_ma.getValue(), rightLDR_ma.getValue(), 
      leftE, rightE,
      std::logf(leftE), std::logf(rightE), PID_output);
  #endif
  Serial.println();
  #if USE_WIFI
    enviarDados();
  #endif
  delay(PID_SAMPLE_RATE);
}
#include <Arduino.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ====================== PINOS ======================
#define LDR1_PIN   39    // A3 — LDR esquerdo (entrada)
#define LDR2_PIN   34    // A2 — LDR direito  (setpoint)
#define MOTOR_OUT  10    // PWM para o motor

// ===================== WI-FI =======================
const char* ssid     = "pi-dashboard";   // rede criada pelo Raspberry Pi (AP)
const char* password = "dashboard123";
const char* serverIP = "http://192.168.4.1:5000/dados";

// ============= CONFIGURAÇÕES DE CONTROLE ===========
#define USE_ACCELEROMETER   0
#define CONTROLL_AXIS       1   // 1 = PID ativo, 0 = saída manual
#define CHANGE_GAIN         0   // 1 = entrada muda os ganhos em tempo real

// ============= CONFIGURAÇÕES DE IMPRESSÃO ==========
#define PRINT_INPUT_INFO    1
#define PRINT_PID_GAINS     1
#define PRINT_SYSTEM_IO     1

// ============= PARÂMETROS AJUSTÁVEIS ===============
#define MOVING_AVERAGE_SAMPLES  5

#define P_GAIN   0.267
#define I_GAIN   0.0
#define D_GAIN   0.0

#define PWM_RESOLUTION  20                    // bits
#define LOWER_CLAMP     0
#define UPPER_CLAMP     ((1 << PWM_RESOLUTION) - 1)   // 1048575
#define PWM_CENTER      (1 << (PWM_RESOLUTION - 1))   // 524288
#define PWM_FREQ        50                    // Hz

#define GAIN_TYPE       0     // 0=Kp  1=Ki  2=Kd
#define GAIN_MULT       2.0

#define SEND_INTERVAL_MS  2000   // envia pro Pi a cada 2 s

// ============= VARIÁVEIS GLOBAIS ===================
double  LDR1_raw, LDR2_raw;          // leituras brutas (0.0–1.0)
double  LDR1_mean, LDR2_mean;        // após média móvel
long    PID_output;                   // saída do PID
double  kp, ki, kd;

long    old_time     = 0;
long    new_time     = 0;
long    last_send_ms = 0;

// ============= OBJETOS =============================
MovingAverage ldr1_filter(MOVING_AVERAGE_SAMPLES, &LDR1_raw, &LDR1_mean);
MovingAverage ldr2_filter(MOVING_AVERAGE_SAMPLES, &LDR2_raw, &LDR2_mean);

// PID: setpoint = LDR2_mean (referência), input = LDR1_mean (posição atual)
PIDController pidCon(P_GAIN, I_GAIN, D_GAIN,
                     &LDR2_mean, &LDR1_mean, &PID_output,
                     LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER);

// ===================================================
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

  double erro = LDR2_mean - LDR1_mean;

  // JSON com todos os dados do sistema
  String payload = "{";
  payload += "\"ldr1_raw\":"   + String(LDR1_raw,   4) + ",";
  payload += "\"ldr2_raw\":"   + String(LDR2_raw,   4) + ",";
  payload += "\"ldr1_mean\":"  + String(LDR1_mean,  4) + ",";
  payload += "\"ldr2_mean\":"  + String(LDR2_mean,  4) + ",";
  payload += "\"erro\":"       + String(erro,        4) + ",";
  payload += "\"pid_output\":" + String(PID_output)     + ",";
  payload += "\"pwm_pct\":"    + String((double)PID_output / UPPER_CLAMP * 100.0, 2);
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

// ===================================================
void setup() {
  Serial.begin(115200);

  analogSetAttenuation(ADC_11db);          // escala completa 0–3.3V no ESP32
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(MOTOR_OUT, PWM_FREQ);
  pinMode(MOTOR_OUT, OUTPUT);
  pinMode(LDR1_PIN,  INPUT);
  pinMode(LDR2_PIN,  INPUT);

  // Leitura inicial para pré-aquecer os filtros
  LDR1_raw = analogReadMilliVolts(LDR1_PIN) / 3300.0;
  LDR2_raw = analogReadMilliVolts(LDR2_PIN) / 3300.0;
  ldr1_filter.update();
  ldr2_filter.update();

  conectarWiFi();

  old_time = millis();
  Serial.println("Setup concluído.");
}

// ===================================================
void loop() {
  new_time = millis();
  double dt = (new_time - old_time) / 1000.0;   // delta em segundos
  if (dt <= 0) dt = 0.001;

  // 1. Leitura dos LDRs em volts normalizados (0.0–1.0)
  LDR1_raw = analogReadMilliVolts(LDR1_PIN) / 3300.0;
  LDR2_raw = analogReadMilliVolts(LDR2_PIN) / 3300.0;

  // 2. Filtro de média móvel
  ldr1_filter.update();
  ldr2_filter.update();

  // 3. Ajuste de ganho em tempo real (opcional)
  #if CHANGE_GAIN
    pidCon.setGain(LDR1_raw * GAIN_MULT, GAIN_TYPE);
  #endif

  // 4. Atualiza o PID
  #if CONTROLL_AXIS
    pidCon.update(dt);
    analogWrite(MOTOR_OUT, (int)PID_output);
  #else
    // modo manual: motor segue LDR1 diretamente
    analogWrite(MOTOR_OUT, (int)(LDR1_raw * UPPER_CLAMP));
  #endif

  // 5. Debug serial
  #if PRINT_INPUT_INFO
    Serial.printf("LDR1_raw=%.3f  LDR2_raw=%.3f  ", LDR1_raw, LDR2_raw);
    Serial.printf("mean1=%.3f  mean2=%.3f  ", LDR1_mean, LDR2_mean);
  #endif
  #if PRINT_PID_GAINS
    pidCon.getAllGains(&kp, &ki, &kd);
    Serial.printf("Kp=%.3f Ki=%.3f Kd=%.3f  ", kp, ki, kd);
  #endif
  #if PRINT_SYSTEM_IO
    Serial.printf("erro=%.3f  PID=%ld  PWM=%.1f%%",
      LDR2_mean - LDR1_mean, PID_output,
      (double)PID_output / UPPER_CLAMP * 100.0);
  #endif
  Serial.println();

  // 6. Envia para o Raspberry Pi a cada SEND_INTERVAL_MS
  if (new_time - last_send_ms >= SEND_INTERVAL_MS) {
    enviarDados();
    last_send_ms = new_time;
  }

  old_time = new_time;
  delay(5);
}

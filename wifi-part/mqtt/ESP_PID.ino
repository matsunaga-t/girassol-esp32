// ================================================================
//  ESP_PID.ino — Sistema de controle PID com LDRs + envio Wi-Fi
//  Conecta ao Access Point do Raspberry Pi e envia dados via HTTP
// ================================================================
#include <Arduino.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ====================== PINOS ===================================
#define LDR1_PIN   39    // GPIO39 (A3) — LDR esquerdo
#define LDR2_PIN   34    // GPIO34 (A2) — LDR direito
#define MOTOR_OUT  10    // GPIO10 — saída PWM para motor

// ====================== WI-FI / SERVIDOR =======================
const char* WIFI_SSID     = "pi-dashboard";            // nome do AP do Pi
const char* WIFI_PASSWORD = "dashboard123";             // senha do AP
const char* SERVER_URL    = "http://192.168.4.1:5000/dados"; // IP fixo do Pi

// ====================== CONFIGURAÇÕES ==========================
#define USE_ACCELEROMETER   0   // 1 para usar o acelerômetro ADXL345
#define CONTROLL_AXIS       1   // 1 para controle PID automático ativo
#define CHANGE_GAIN         0   // 1 para ajustar ganhos pela entrada

#define PRINT_INPUT_INFO    1   // imprime leituras dos LDRs no serial
#define PRINT_PID_GAINS     1   // imprime ganhos Kp, Ki, Kd no serial
#define PRINT_SYSTEM_IO     1   // imprime entrada/saída do PID no serial

// ====================== VALORES AJUSTÁVEIS =====================
#define MOVING_AVERAGE_SAMPLES  5       // amostras da média móvel
#define SEND_INTERVAL_MS        2000    // intervalo de envio ao Pi (ms)
#define PWM_FREQ_HZ             50      // frequência PWM (50 Hz para servo)
#define PWM_RESOLUTION_BITS     20      // resolução PWM em bits

// ====================== GANHOS PID =============================
#define P_GAIN  0.267
#define I_GAIN  0.0
#define D_GAIN  0.0

// ====================== LIMITES PWM ============================
#define LOWER_CLAMP   0
#define UPPER_CLAMP   ((1 << PWM_RESOLUTION_BITS) - 1)  // 1048575 (20 bits)
#define PWM_CENTER    (1 << (PWM_RESOLUTION_BITS - 1))  // 524288  (centro)

// ====================== VARIÁVEIS GLOBAIS ======================
double  LDR1_raw,  LDR2_raw;    // leituras brutas (0.0–1.0 normalizado)
double  LDR1_mean, LDR2_mean;   // médias móveis filtradas
long    PID_output;              // saída do controlador PID
double  pid_error;               // erro atual (setpoint - input)
double  kp, ki, kd;              // cópia dos ganhos para impressão

long    old_time = 0, new_time;  // controle de tempo do loop
long    last_send_time = 0;      // controle do intervalo de envio HTTP

// ====================== OBJETOS ================================
MovingAverage filter_ldr1(MOVING_AVERAGE_SAMPLES, &LDR1_raw,  &LDR1_mean);
MovingAverage filter_ldr2(MOVING_AVERAGE_SAMPLES, &LDR2_raw,  &LDR2_mean);

// PID: setpoint = LDR2_mean, input = LDR1_mean, saída em PID_output
PIDController pidCon(
  P_GAIN, I_GAIN, D_GAIN,
  &LDR2_mean,   // setpoint: nível desejado (LDR direito)
  &LDR1_mean,   // input:    leitura atual  (LDR esquerdo)
  &PID_output,
  LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER
);

// ====================== SETUP ==================================
void setup() {
  Serial.begin(115200);

  // configura pinos
  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);
  pinMode(MOTOR_OUT, OUTPUT);
  analogWriteFrequency(MOTOR_OUT, PWM_FREQ_HZ);
  analogWriteResolution(PWM_RESOLUTION_BITS);

  // inicializa ADC com atenuação máxima (0–3.3V)
  analogSetAttenuation(ADC_11db);

  // conecta ao AP do Raspberry Pi
  Serial.print("Conectando ao AP: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado! IP do ESP32: ");
  Serial.println(WiFi.localIP());

  #if USE_ACCELEROMETER
    if (!accel.begin()) {
      Serial.println("ADXL345 não encontrado. Verifique as conexões!");
      while (1);
    }
    accel.setRange(ADXL345_RANGE_2_G);
  #endif
}

// ====================== ENVIO HTTP =============================
void enviarDados() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  // monta JSON com todos os campos
  String payload = "{";
  payload += "\"ldr1_raw\":"   + String(LDR1_raw,   4) + ",";
  payload += "\"ldr2_raw\":"   + String(LDR2_raw,   4) + ",";
  payload += "\"ldr1_mean\":"  + String(LDR1_mean,  4) + ",";
  payload += "\"ldr2_mean\":"  + String(LDR2_mean,  4) + ",";
  payload += "\"pid_error\":"  + String(pid_error,  4) + ",";
  payload += "\"pid_output\":" + String(PID_output)    + ",";
  payload += "\"pwm_value\":"  + String(PID_output);   // mesmo valor do PWM escrito
  payload += "}";

  int code = http.POST(payload);
  if (code != 200) {
    Serial.print("[HTTP erro] código: ");
    Serial.println(code);
  }
  http.end();
}

// ====================== LOOP ===================================
void loop() {
  new_time = millis();
  double timeDelta = (new_time - old_time) / 1000.0; // converte ms → segundos

  // --- leitura dos LDRs (normalizado 0.0–1.0 com calibração por curva) ---
  double v1 = analogReadMilliVolts(LDR1_PIN) / 1000.0;
  double v2 = analogReadMilliVolts(LDR2_PIN) / 1000.0;
  LDR1_raw = 0.831185 * pow(v1, 1.65604);  // curva de calibração (ADC_test.ino)
  LDR2_raw = 0.424298 * pow(v2, 2.22366);

  // --- filtro de média móvel ---
  filter_ldr1.update();
  filter_ldr2.update();

  // --- calcula erro atual (antes do update do PID) ---
  pid_error = LDR2_mean - LDR1_mean;

  // --- controle PID ---
  #if CONTROLL_AXIS
    pidCon.update(timeDelta);
    analogWrite(MOTOR_OUT, (int)PID_output);
  #else
    // modo manual: LDR1 controla diretamente o motor
    analogWrite(MOTOR_OUT, (int)(LDR1_raw * UPPER_CLAMP));
  #endif

  // --- debug serial ---
  #if PRINT_INPUT_INFO
    Serial.print("LDR1=");  Serial.print(LDR1_raw,  3);
    Serial.print(" LDR2="); Serial.print(LDR2_raw,  3);
    Serial.print(" m1=");   Serial.print(LDR1_mean, 3);
    Serial.print(" m2=");   Serial.print(LDR2_mean, 3);
  #endif

  #if PRINT_PID_GAINS
    pidCon.getAllGains(&kp, &ki, &kd);
    Serial.print(" | Kp="); Serial.print(kp, 3);
    Serial.print(" Ki=");   Serial.print(ki, 3);
    Serial.print(" Kd=");   Serial.print(kd, 3);
  #endif

  #if PRINT_SYSTEM_IO
    Serial.print(" | err="); Serial.print(pid_error,  3);
    Serial.print(" out=");   Serial.print(PID_output);
  #endif

  Serial.println();

  // --- envia dados ao Raspberry Pi a cada SEND_INTERVAL_MS ---
  if (new_time - last_send_time >= SEND_INTERVAL_MS) {
    enviarDados();
    last_send_time = new_time;
  }

  delay(5);
  old_time = new_time;
}

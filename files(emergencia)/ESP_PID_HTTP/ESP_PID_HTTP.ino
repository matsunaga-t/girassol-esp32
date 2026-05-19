// ================================================================
//  ESP_PID_HTTP.ino — Versão hotspot do celular
//  Conecta na rede do celular e envia dados HTTP para o Pi.
//
//  ANTES DE CARREGAR:
//  1. Preencha WIFI_SSID e WIFI_PASSWORD com os dados do hotspot
//  2. Ligue o Pi, rode servidor_http.py e anote o IP exibido
//  3. Cole o IP em SERVER_IP abaixo
// ================================================================
#include <Arduino.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ====================== WI-FI DO CELULAR =======================
const char* WIFI_SSID     = "NOME_DO_HOTSPOT";   // ← nome da rede do celular
const char* WIFI_PASSWORD = "SENHA_DO_HOTSPOT";  // ← senha do hotspot

// ====================== IP DO RASPBERRY PI =====================
// Descubra rodando: python3 servidor_http.py no Pi
// O IP aparece no terminal ao iniciar. Ex: 192.168.43.105
const char* SERVER_IP  = "192.168.X.X";          // ← IP do Pi na rede do celular
const int   SERVER_PORT = 5000;

// URL montada a partir do IP acima
String SERVER_URL;  // preenchida no setup()

// ====================== PINOS ==================================
#define LDR1_PIN    39
#define LDR2_PIN    34
#define SOLAR_V_PIN 32
#define SOLAR_A_PIN 33
#define MOTOR_OUT   10

// ====================== CALIBRAÇÃO SOLAR =======================
#define SOLAR_V_FACTOR      5.0f
#define SOLAR_V_VREF        3.3f
#define SOLAR_A_OFFSET      2.5f
#define SOLAR_A_SENSITIVITY 0.1f

// ====================== CONFIGURAÇÕES ==========================
#define MOVING_AVERAGE_SAMPLES  5
#define SEND_INTERVAL_MS        2000
#define PWM_FREQ_HZ             50
#define PWM_RESOLUTION_BITS     20

#define P_GAIN  0.267
#define I_GAIN  0.0
#define D_GAIN  0.0

#define LOWER_CLAMP   0
#define UPPER_CLAMP   ((1 << PWM_RESOLUTION_BITS) - 1)
#define PWM_CENTER    (1 << (PWM_RESOLUTION_BITS - 1))

// ====================== VARIÁVEIS ==============================
double LDR1_raw,  LDR2_raw;
double LDR1_mean, LDR2_mean;
long   PID_output;
double pid_error;
float  solar_v = 0.0f;
float  solar_a = 0.0f;

long old_time  = 0;
long last_send = 0;

MovingAverage filter_ldr1(MOVING_AVERAGE_SAMPLES, &LDR1_raw,  &LDR1_mean);
MovingAverage filter_ldr2(MOVING_AVERAGE_SAMPLES, &LDR2_raw,  &LDR2_mean);

PIDController pidCon(
  P_GAIN, I_GAIN, D_GAIN,
  &LDR2_mean, &LDR1_mean, &PID_output,
  LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER
);

// ====================== LEITURA SOLAR ==========================
void lerPainelSolar() {
  float v_adc = analogReadMilliVolts(SOLAR_V_PIN) / 1000.0f;
  solar_v = v_adc * SOLAR_V_FACTOR;
  float a_adc = analogReadMilliVolts(SOLAR_A_PIN) / 1000.0f;
  solar_a = (a_adc - SOLAR_A_OFFSET) / SOLAR_A_SENSITIVITY;
  if (solar_a < 0) solar_a = 0;
}

// ====================== WI-FI ==================================
void conectarWiFi() {
  Serial.print("[WiFi] conectando em "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (++tentativas > 30) {   // 15s sem conectar → reinicia
      Serial.println("\n[WiFi] timeout — reiniciando ESP32");
      ESP.restart();
    }
  }
  Serial.println("\n[WiFi] conectado! IP do ESP32: " + WiFi.localIP().toString());
  Serial.println("[WiFi] enviando para: " + SERVER_URL);
}

// ====================== SETUP ==================================
void setup() {
  Serial.begin(115200);

  // monta a URL com o IP configurado
  SERVER_URL = "http://" + String(SERVER_IP) + ":" +
               String(SERVER_PORT) + "/dados";

  pinMode(LDR1_PIN,    INPUT);
  pinMode(LDR2_PIN,    INPUT);
  pinMode(SOLAR_V_PIN, INPUT);
  pinMode(SOLAR_A_PIN, INPUT);
  pinMode(MOTOR_OUT,   OUTPUT);
  analogWriteFrequency(MOTOR_OUT, PWM_FREQ_HZ);
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogSetAttenuation(ADC_11db);

  conectarWiFi();
}

// ====================== ENVIO HTTP =============================
void enviarDados() {
  if (WiFi.status() != WL_CONNECTED) { conectarWiFi(); return; }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);  // 3s timeout — evita travar o loop

  String payload = "{";
  payload += "\"solar_v\":"    + String(solar_v,   3) + ",";
  payload += "\"solar_a\":"    + String(solar_a,   3) + ",";
  payload += "\"ldr1_raw\":"   + String(LDR1_raw,  4) + ",";
  payload += "\"ldr2_raw\":"   + String(LDR2_raw,  4) + ",";
  payload += "\"ldr1_mean\":"  + String(LDR1_mean, 4) + ",";
  payload += "\"ldr2_mean\":"  + String(LDR2_mean, 4) + ",";
  payload += "\"pid_error\":"  + String(pid_error, 4) + ",";
  payload += "\"pid_output\":" + String(PID_output)   + ",";
  payload += "\"pwm_value\":"  + String(PID_output);
  payload += "}";

  int code = http.POST(payload);
  Serial.println(code == 200 ? "[HTTP] OK" : "[HTTP] erro " + String(code));
  http.end();
}

// ====================== LOOP ===================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();

  long new_time    = millis();
  double timeDelta = (new_time - old_time) / 1000.0;

  // LDRs
  double v1 = analogReadMilliVolts(LDR1_PIN) / 1000.0;
  double v2 = analogReadMilliVolts(LDR2_PIN) / 1000.0;
  LDR1_raw  = 0.831185 * pow(v1, 1.65604);
  LDR2_raw  = 0.424298 * pow(v2, 2.22366);
  filter_ldr1.update();
  filter_ldr2.update();

  lerPainelSolar();

  pid_error = LDR2_mean - LDR1_mean;
  pidCon.update(timeDelta);
  analogWrite(MOTOR_OUT, (int)PID_output);

  Serial.printf("V=%.2fV A=%.3fA W=%.2fW | ldr1=%.3f ldr2=%.3f err=%.3f out=%ld\n",
    solar_v, solar_a, solar_v * solar_a,
    LDR1_raw, LDR2_raw, pid_error, PID_output);

  if (new_time - last_send >= SEND_INTERVAL_MS) {
    enviarDados();
    last_send = new_time;
  }

  delay(5);
  old_time = new_time;
}

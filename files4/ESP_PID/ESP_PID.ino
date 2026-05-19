// ================================================================
//  ESP_PID.ino — PID com LDRs + painel solar (V e A) + MQTT
//  Tópico publish : pid/dados
//  Tópico subscribe: pid/config
// ================================================================
#include <Arduino.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <WiFi.h>
#include <PubSubClient.h>

// ====================== WI-FI / MQTT ===========================
const char* WIFI_SSID     = "pi-dashboard";
const char* WIFI_PASSWORD = "dashboard123";
const char* MQTT_BROKER   = "192.168.4.1";
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "esp32-pid";
const char* TOPIC_PUBLISH = "pid/dados";
const char* TOPIC_CONFIG  = "pid/config";

// ====================== PINOS ==================================
#define LDR1_PIN    39    // GPIO39 (A3) — LDR esquerdo
#define LDR2_PIN    34    // GPIO34 (A2) — LDR direito
#define SOLAR_V_PIN 32    // GPIO32 (A4) — divisor de tensão do painel
#define SOLAR_A_PIN 33    // GPIO33 (A5) — saída do sensor de corrente
#define MOTOR_OUT   10    // GPIO10 — saída PWM motor

// ====================== CALIBRAÇÃO SOLAR =======================
// Divisor de tensão: R1=30kΩ, R2=7.5kΩ → fator = (R1+R2)/R2 = 5
// Ajuste SOLAR_V_FACTOR conforme seu divisor real
#define SOLAR_V_FACTOR    5.0f    // multiplica a leitura para obter V real
#define SOLAR_V_VREF      3.3f    // tensão de referência do ADC (V)

// Sensor de corrente (ex: ACS712-20A): offset=2.5V, sensibilidade=100mV/A
// Ajuste SOLAR_A_OFFSET e SOLAR_A_SENSITIVITY conforme seu sensor
#define SOLAR_A_OFFSET        2.5f   // tensão de saída com 0A (V)
#define SOLAR_A_SENSITIVITY   0.1f   // V por Ampere (100mV/A para ACS712-20A)

// ====================== CONFIGURAÇÕES ==========================
#define MOVING_AVERAGE_SAMPLES  5
#define PUBLISH_INTERVAL_MS     2000
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

float  solar_v = 0.0f;   // tensão do painel (V)
float  solar_a = 0.0f;   // corrente do painel (A)

long old_time    = 0;
long last_publish = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

MovingAverage filter_ldr1(MOVING_AVERAGE_SAMPLES, &LDR1_raw,  &LDR1_mean);
MovingAverage filter_ldr2(MOVING_AVERAGE_SAMPLES, &LDR2_raw,  &LDR2_mean);

PIDController pidCon(
  P_GAIN, I_GAIN, D_GAIN,
  &LDR2_mean, &LDR1_mean, &PID_output,
  LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER
);

// ====================== LEITURA SOLAR ==========================
void lerPainelSolar() {
  // tensão: leitura ADC → mV → V → desnormaliza pelo divisor
  float v_adc = analogReadMilliVolts(SOLAR_V_PIN) / 1000.0f;
  solar_v = v_adc * SOLAR_V_FACTOR;

  // corrente: tensão do sensor → subtrai offset → divide sensibilidade
  float a_adc = analogReadMilliVolts(SOLAR_A_PIN) / 1000.0f;
  solar_a = (a_adc - SOLAR_A_OFFSET) / SOLAR_A_SENSITIVITY;
  if (solar_a < 0) solar_a = 0;  // descarta negativo (ruído)
}

// ====================== MQTT CALLBACK ==========================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("[MQTT config] "); Serial.println(msg);

  double new_kp=-1, new_ki=-1, new_kd=-1;
  int pos=0;
  while (pos < (int)msg.length()) {
    int eq  = msg.indexOf('=', pos);
    int sep = msg.indexOf(',', pos);
    if (sep==-1) sep=msg.length();
    if (eq==-1)  break;
    String key = msg.substring(pos, eq); key.trim();
    float  val = msg.substring(eq+1, sep).toFloat();
    if (key=="kp") new_kp=val;
    if (key=="ki") new_ki=val;
    if (key=="kd") new_kd=val;
    pos=sep+1;
  }
  double kp,ki,kd;
  pidCon.getAllGains(&kp,&ki,&kd);
  pidCon.setAllGains(new_kp>=0?new_kp:kp, new_ki>=0?new_ki:ki, new_kd>=0?new_kd:kd);
  pidCon.reset();
  Serial.println("[PID] ganhos atualizados.");
}

void conectarWiFi() {
  Serial.print("Wi-Fi "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status()!=WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: "+WiFi.localIP().toString());
}

void conectarMQTT() {
  while (!mqtt.connected()) {
    Serial.print("MQTT...");
    if (mqtt.connect(MQTT_CLIENT)) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_CONFIG);
    } else {
      Serial.print("falhou rc="); Serial.print(mqtt.state());
      Serial.println(". 3s..."); delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LDR1_PIN,    INPUT);
  pinMode(LDR2_PIN,    INPUT);
  pinMode(SOLAR_V_PIN, INPUT);
  pinMode(SOLAR_A_PIN, INPUT);
  pinMode(MOTOR_OUT,   OUTPUT);
  analogWriteFrequency(MOTOR_OUT, PWM_FREQ_HZ);
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogSetAttenuation(ADC_11db);

  conectarWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  conectarMQTT();
}

void publicarDados() {
  String p = "{";
  p += "\"solar_v\":"   + String(solar_v,  3) + ",";
  p += "\"solar_a\":"   + String(solar_a,  3) + ",";
  p += "\"ldr1_raw\":"  + String(LDR1_raw, 4) + ",";
  p += "\"ldr2_raw\":"  + String(LDR2_raw, 4) + ",";
  p += "\"ldr1_mean\":" + String(LDR1_mean,4) + ",";
  p += "\"ldr2_mean\":" + String(LDR2_mean,4) + ",";
  p += "\"pid_error\":" + String(pid_error,4) + ",";
  p += "\"pid_output\":" + String(PID_output) + ",";
  p += "\"pwm_value\":" + String(PID_output);
  p += "}";
  mqtt.publish(TOPIC_PUBLISH, p.c_str())
    ? Serial.println("[MQTT] OK")
    : Serial.println("[MQTT] falha");
}

void loop() {
  if (WiFi.status()!=WL_CONNECTED) conectarWiFi();
  if (!mqtt.connected())           conectarMQTT();
  mqtt.loop();

  long new_time    = millis();
  double timeDelta = (new_time - old_time) / 1000.0;

  // LDRs
  double v1 = analogReadMilliVolts(LDR1_PIN) / 1000.0;
  double v2 = analogReadMilliVolts(LDR2_PIN) / 1000.0;
  LDR1_raw  = 0.831185 * pow(v1, 1.65604);
  LDR2_raw  = 0.424298 * pow(v2, 2.22366);
  filter_ldr1.update();
  filter_ldr2.update();

  // Painel solar
  lerPainelSolar();

  // PID
  pid_error = LDR2_mean - LDR1_mean;
  pidCon.update(timeDelta);
  analogWrite(MOTOR_OUT, (int)PID_output);

  Serial.printf("V=%.2fV A=%.3fA W=%.2fW | ldr1=%.3f ldr2=%.3f err=%.3f out=%ld\n",
    solar_v, solar_a, solar_v*solar_a,
    LDR1_raw, LDR2_raw, pid_error, PID_output);

  if (new_time - last_publish >= PUBLISH_INTERVAL_MS) {
    publicarDados();
    last_publish = new_time;
  }

  delay(5);
  old_time = new_time;
}

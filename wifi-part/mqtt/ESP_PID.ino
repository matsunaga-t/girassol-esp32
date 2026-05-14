// ================================================================
//  ESP_PID.ino — Controle PID com LDRs + comunicação MQTT
//  Publica dados em pid/dados
//  Recebe ajuste de ganhos em pid/config
// ================================================================
#include <Arduino.h>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <WiFi.h>
#include <PubSubClient.h>   // instale via Arduino Library Manager

// ====================== WI-FI / MQTT ===========================
const char* WIFI_SSID     = "pi-dashboard";
const char* WIFI_PASSWORD = "dashboard123";
const char* MQTT_BROKER   = "192.168.4.1";
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "esp32-pid";

// ====================== TÓPICOS ================================
const char* TOPIC_PUBLISH = "pid/dados";   // ESP32 → broker → Flask
const char* TOPIC_CONFIG  = "pid/config";  // Flask → broker → ESP32

// ====================== PINOS ==================================
#define LDR1_PIN   39
#define LDR2_PIN   34
#define MOTOR_OUT  10

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
long   old_time = 0, last_publish = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

MovingAverage filter_ldr1(MOVING_AVERAGE_SAMPLES, &LDR1_raw,  &LDR1_mean);
MovingAverage filter_ldr2(MOVING_AVERAGE_SAMPLES, &LDR2_raw,  &LDR2_mean);

PIDController pidCon(
  P_GAIN, I_GAIN, D_GAIN,
  &LDR2_mean, &LDR1_mean, &PID_output,
  LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER
);

// ====================== CALLBACK MQTT ==========================
// Chamado ao receber mensagem em pid/config
// Formato: "kp=0.5,ki=0.01,kd=0.001"
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("[MQTT config] "); Serial.println(msg);

  double new_kp = -1, new_ki = -1, new_kd = -1;
  int pos = 0;
  while (pos < (int)msg.length()) {
    int eq  = msg.indexOf('=', pos);
    int sep = msg.indexOf(',', pos);
    if (sep == -1) sep = msg.length();
    if (eq  == -1) break;
    String key = msg.substring(pos, eq); key.trim();
    float  val = msg.substring(eq + 1, sep).toFloat();
    if (key == "kp") new_kp = val;
    if (key == "ki") new_ki = val;
    if (key == "kd") new_kd = val;
    pos = sep + 1;
  }

  double kp, ki, kd;
  pidCon.getAllGains(&kp, &ki, &kd);
  pidCon.setAllGains(
    new_kp >= 0 ? new_kp : kp,
    new_ki >= 0 ? new_ki : ki,
    new_kd >= 0 ? new_kd : kd
  );
  pidCon.reset();
  Serial.println("[PID] ganhos atualizados e controlador reiniciado.");
}

void conectarWiFi() {
  Serial.print("Wi-Fi "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: " + WiFi.localIP().toString());
}

void conectarMQTT() {
  while (!mqtt.connected()) {
    Serial.print("MQTT...");
    if (mqtt.connect(MQTT_CLIENT)) {
      Serial.println("OK");
      mqtt.subscribe(TOPIC_CONFIG);
    } else {
      Serial.print("falhou rc="); Serial.print(mqtt.state());
      Serial.println(". 3s...");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LDR1_PIN,  INPUT);
  pinMode(LDR2_PIN,  INPUT);
  pinMode(MOTOR_OUT, OUTPUT);
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
  double kp, ki, kd;
  pidCon.getAllGains(&kp, &ki, &kd);
  String p = "{";
  p += "\"ldr1_raw\":"   + String(LDR1_raw,  4) + ",";
  p += "\"ldr2_raw\":"   + String(LDR2_raw,  4) + ",";
  p += "\"ldr1_mean\":"  + String(LDR1_mean, 4) + ",";
  p += "\"ldr2_mean\":"  + String(LDR2_mean, 4) + ",";
  p += "\"pid_error\":"  + String(pid_error, 4) + ",";
  p += "\"pid_output\":" + String(PID_output)   + ",";
  p += "\"pwm_value\":"  + String(PID_output)   + ",";
  p += "\"kp\":"         + String(kp, 4)        + ",";
  p += "\"ki\":"         + String(ki, 4)        + ",";
  p += "\"kd\":"         + String(kd, 4);
  p += "}";
  mqtt.publish(TOPIC_PUBLISH, p.c_str())
    ? Serial.println("[MQTT] OK")
    : Serial.println("[MQTT] falha");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWiFi();
  if (!mqtt.connected())             conectarMQTT();
  mqtt.loop();

  long new_time    = millis();
  double timeDelta = (new_time - old_time) / 1000.0;

  double v1 = analogReadMilliVolts(LDR1_PIN) / 1000.0;
  double v2 = analogReadMilliVolts(LDR2_PIN) / 1000.0;
  LDR1_raw  = 0.831185 * pow(v1, 1.65604);
  LDR2_raw  = 0.424298 * pow(v2, 2.22366);

  filter_ldr1.update();
  filter_ldr2.update();
  pid_error = LDR2_mean - LDR1_mean;
  pidCon.update(timeDelta);
  analogWrite(MOTOR_OUT, (int)PID_output);

  Serial.printf("LDR1=%.3f LDR2=%.3f err=%.3f out=%ld\n",
    LDR1_raw, LDR2_raw, pid_error, PID_output);

  if (new_time - last_publish >= PUBLISH_INTERVAL_MS) {
    publicarDados();
    last_publish = new_time;
  }

  delay(5);
  old_time = new_time;
}

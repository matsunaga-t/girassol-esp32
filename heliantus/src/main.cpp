/* Standard libraries */
#include <arduino.h>
#include <cmath>
#include "esp32-hal.h"
#include "signalConditioning.h"
#include "controlTheory.h"
#include <HTTPClient.h>
#include <WiFi.h>

/* Third party libraries */
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <PubSubClient.h>

/* Settings header */
#include "projectSettings.h"
#include "wifiDefines.h"

// definições para o programa
#define X2 X
#define X(LDR_pin, LDR_number, LDR_raw, LDR_ms, LDR_norm, LDR_mean, LDR_E, total_E) \
    uint32_t LDR_raw;   \
    float    LDR_E;  \
    float    LDR_mean; \
    /* inicialização das classes */  \
    MultisampleManager LDR_ms;  \
    LDRNormalizer LDR_norm(LDR_PARAMS(LDR_number));
XTABLE_LDR
#undef X
#undef X2

MultisampleManager solarPanel_ms;

float leftE, rightE;                     // Iluminância dos LDRs
long PID_output;                         // saída do PID
long long nextControllTime_us = PID_CONTROLL_DELAY * 1000;
long long nextSendTime_us     = WIFI_SEND_DELAY * 1000;

Servo motor;
#if CHANGE_PID_GAIN
    float kp, ki, kd;                      // Ganhos do PID
    float input1;
    float input2;
    float input3;
#endif
#if ENABLE_HALT
    bool running = false;
#endif
    
#if USE_ACCELEROMETER
    Adafruit_Sensor *mpuAccelSensor;
    Adafruit_MPU6050 mpuSensor;
    float planeAngle_rad;
#endif

PIDController pidCon = PIDController(P_GAIN, I_GAIN, D_GAIN, &leftE, &rightE, &PID_output, LOWER_CLAMP, UPPER_CLAMP, PWM_CENTER);

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

void conectarWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    #if PRINT_DEBUG
        Serial.print("Conectando ao wi-fi");
    #endif
    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
        delay(500);
        #if PRINT_DEBUG
            Serial.print(".");
        #endif
        tentativas++;
    }
    #if PRINT_DEBUG
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWi-Fi conectado! IP: " + WiFi.localIP().toString());
        } else {
            Serial.println("\nFalha no Wi-Fi — continuando sem envio de dados.");
        }
    #endif
}

void enviarDados() {
    if (WiFi.status() != WL_CONNECTED) return;

    // JSON com todos os dados do sistema

    String payload = "{";
    payload += "\"solar_v\":"    + String(solarPanel_ms.getAverage() * 2,   3) + ",";
    payload += "\"solar_a\":"    + String(solarPanel_ms.getAverage() / .1,   3) + ",";
    payload += "\"ldr1_raw\":"   + String(leftLDR_raw / 1000.0f,  4) + ",";
    payload += "\"ldr2_raw\":"   + String(rightLDR_raw / 1000.0f,  4) + ",";
    payload += "\"ldr1_mean\":"  + String(leftLDR_ms.getAverage(), 4) + ",";
    payload += "\"ldr2_mean\":"  + String(rightLDR_ms.getAverage(), 4) + ",";
    payload += "\"pid_error\":"  + String(leftE - rightE, 4) + ",";
    payload += "\"pid_output\":" + String(PID_output)   + ",";
    payload += "\"pwm_value\":"  + String(PID_output);
    payload += "}";

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    //http.setTimeout(3000);  // 3s timeout — evita travar o loop
    int code = http.POST(payload);
    
    #if PRINT_DEBUG
        if (code != 200) {
            Serial.print("[HTTP] erro: ");
            Serial.println(code);
        }
    #endif
    http.end();
}

void setup() {
    Serial.begin(115200);

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
    
    pinMode(SOLAR_PIN, INPUT);

    #if USE_ACCELEROMETER
        if (!mpuSensor.begin()) {
            Serial.println("Não foi possível encontrar o MPU6050. Verifique as conexões!");
            while (1);
        }
        mpuSensor.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpuAccelSensor = mpuSensor.getAccelerometerSensor();
    #endif

    #if CHANGE_PID_GAIN
        pinMode(INPUT1_PIN, INPUT);
        pinMode(INPUT2_PIN, INPUT);
        pinMode(INPUT3_PIN, INPUT);
    #endif
    
    #if ENABLE_HALT
        pinMode(HALT_PIN, INPUT_PULLDOWN);
    #endif
        
    #if USE_WIFI
        conectarWiFi();
    #endif
}

void loop(){
    #if USE_ACCELEROMETER
        sensors_event_t accelarationEvent;
        mpuAccelSensor->getEvent(&accelarationEvent);
        planeAngle_rad = atan2f(accelarationEvent.acceleration.ACCEL_PARALLEL_AXIS, accelarationEvent.acceleration.ACCEL_NORMAL_AXIS);
        float pwmChange = planeAngle_rad / (MAXIMUM_ANGLE * DEG_TO_RAD);
        if(pwmChange > 0) {
            if(pwmChange > 1)
                pwmChange = 1;
            pidCon.setLimits(LOWER_CLAMP, UPPER_CLAMP - pwmChange * (UPPER_CLAMP - PWM_CENTER));
        }
        else if(pwmChange < 0) {
            if(pwmChange < -1)
                pwmChange = -1;
            pidCon.setLimits(LOWER_CLAMP - pwmChange * (LOWER_CLAMP - PWM_CENTER), UPPER_CLAMP);
        }
    #endif
    
    #if ENABLE_HALT
        running = running ^ digitalRead(HALT_PIN);
    #endif
    
    #if CHANGE_PID_GAIN
        input1 = (analogReadMilliVolts(INPUT1_PIN) - 142) / (3165.0 - 142.0);
        input2 = (analogReadMilliVolts(INPUT2_PIN) - 142) / (3165.0 - 142.0);
        input3 = (analogReadMilliVolts(INPUT3_PIN) - 142) / (3165.0 - 142.0);

        pidCon.setAllGains(input1 * P_GAIN_MULT, input2 * I_GAIN_MULT, input3 * D_GAIN_MULT);
    #endif

    // Imprime a entrada do potenciômetro
    #if PRINT_INPUT_INFO
        #if CHANGE_PID_GAIN
            Serial.printf("[%5.3f, %5.3f, %5.3f] ", input1, input2, input3);
        #endif
        #if ENABLE_HALT
            Serial.printf("%s  ", running ? "RUNNING" : "HALTED ");
        #endif
    #endif

    // Ganhos do PID
    #if PRINT_PID_GAINS
        pidCon.getAllGains(&kp, &ki, &kd);
        Serial.printf("PID = {%5.3f, %5.3f, %5.3f}  ", kp, ki, kd);
    #endif
    /* Original
    leftE = rightE = 0.0f;
    #define X2 X
    #define X(LDR_pin, LDR_number, LDR_raw, LDR_ms, LDR_norm, LDR_mean, LDR_E, total_E) \
        LDR_raw = analogReadMilliVolts(LDR_pin);  \
        LDR_ms.addSample(LDR_raw / 1000.0f);  \
        LDR_E = LDR_norm.toIlluminance(LDR_ms.getAverage());  \
        total_E += LDR_E;
    XTABLE_LDR
    #undef X
    #undef X2
    PID_output = pidCon.update();
    */

    #define X2 X
    #define X(LDR_pin, LDR_number, LDR_raw, LDR_ms, LDR_norm, LDR_mean, LDR_E, total_E) \
        LDR_raw = analogReadMilliVolts(LDR_pin);  \
        LDR_ms.addSample(LDR_raw / 1000.0f);
    XTABLE_LDR
    #undef X
    #undef X2
    
    solarPanel_ms.addSample(analogReadMilliVolts(SOLAR_PIN) / 1000.0);
    
    #if PRINT_SYSTEM_IO
        #if USE_ACCELEROMETER
            Serial.printf("theta = %4.1f° ", planeAngle_rad * RAD_TO_DEG);
        #endif
        Serial.printf("raw = (%4d, %4d)  ms = (%5.2f, %5.2f)  E = (%7.0f, %7.0f)  ln(E) = (%5.2f, %5.2f) -> %d / sol = %5.2f",
            leftLDR_raw, rightLDR_raw,
            leftLDR_ms.getAverage(), rightLDR_ms.getAverage(), 
            leftE, rightE,
            logf(leftE), logf(rightE), PID_output, solarPanel_ms.getAverage());
    #endif
    
    #if USE_WIFI
        if(esp_timer_get_time() > nextControllTime_us){
            nextSendTime_us += WIFI_SEND_DELAY * 1000;
            enviarDados();
            solarPanel_ms.reset();
        }
    #endif
    
    if(esp_timer_get_time() > nextControllTime_us){
        nextControllTime_us += PID_CONTROLL_DELAY * 1000;
        leftE = rightE = 0.0f;
        #define X2 X
        #define X(LDR_pin, LDR_number, LDR_raw, LDR_ms, LDR_norm, LDR_mean, LDR_E, total_E) \
            LDR_mean = LDR_ms.getAverage(); \
            LDR_E = LDR_norm.toIlluminance(LDR_mean);  \
            total_E += LDR_E; \
            LDR_ms.reset();
        XTABLE_LDR
        #undef X
        #undef X2
        PID_output = pidCon.update();
    }
    
    #if ENABLE_HALT
    if(running)
    motor.write(PID_output);
    else
    motor.write(1500);
    #endif
    #if PRINT_INPUT_INFO | PRINT_PID_GAINS  | PRINT_SYSTEM_IO  | PRINT_DEBUG 
        Serial.println();
    #endif
}
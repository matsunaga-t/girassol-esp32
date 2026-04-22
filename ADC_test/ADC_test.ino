#include <Arduino.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <cmath>

#define ADC1 32
#define ADC2 33
#define ADC3 34
#define ADC4 35
#define RAW_MODE_SELECT 14
#define ALTERNATE_3V3 26

void setup() {
  // put your setup code here, to run once:
  pinMode(ADC1, INPUT);
  pinMode(ADC2, INPUT);
  pinMode(ADC3, INPUT);
  pinMode(ADC4, INPUT);
  pinMode(RAW_MODE_SELECT, INPUT_PULLDOWN);
  pinMode(ALTERNATE_3V3, OUTPUT);
  digitalWrite(ALTERNATE_3V3, HIGH);

  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
}

double adc1, adc2, adc3, adc4;

void loop() {
  // put your main code here, to run repeatedly:
  adc1 = analogReadMilliVolts(ADC1) / 1000.0;
  adc2 = analogReadMilliVolts(ADC2) / 1000.0;
  adc3 = analogReadMilliVolts(ADC3) / 1000.0;
  adc4 = analogReadMilliVolts(ADC4) / 1000.0;

  if(!digitalRead(RAW_MODE_SELECT)){
    adc1 = 0.475537 * (pow(adc1, 2.11371));
    adc2 = 0.522091 * (pow(adc2, 2.04352));
    adc3 = 0.831185 * (pow(adc3, 1.65604));
    adc4 = 0.424298 * (pow(adc4, 2.22366));
  }

  double mean = (adc1 + adc2 + adc3 + adc4) / 4;
  Serial.printf("adc1=% 5.3lf    adc2=% 5.3lf    adc3=% 5.3lf    adc4=% 5.3lf    \n", adc1 - mean, adc2 - mean, adc3 - mean, adc4 - mean);
}

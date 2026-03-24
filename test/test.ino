#include <Arduino.h>
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define INPUT_PIN 35
#define ALTERNATE_3V3 25

void setup() {
  // put your setup code here, to run once:
  pinMode(INPUT_PIN, INPUT);
  pinMode(ALTERNATE_3V3, OUTPUT);
  digitalWrite(ALTERNATE_3V3, HIGH)

  Serial.begin(115200);

  while(!Serial);

  delay(1000);

  adc_cali_scheme_ver_t caliSchemeVal;
  adc_cali_check_scheme(&caliSchemeVal);

  adc_cali_line_fitting_efuse_val_t efuseVal;
  adc_cali_scheme_line_fitting_check_efuse(&efuseVal);

  switch(caliSchemeVal){
    case ADC_CALI_SCHEME_VER_LINE_FITTING:
    Serial.println("line fitting supported.");
    break;

    case ADC_CALI_SCHEME_VER_CURVE_FITTING:
    Serial.println("curve fitting supported.");
    break;

    default:
    Serial.println("error.");
    break;
  }

  switch(efuseVal){
    case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
    Serial.println("Vref e-fuse.");
    break;

    case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
    Serial.println("two point e-fuse.");
    break;

    case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
    Serial.println("no e-fuse.");
    break;
  }

  analogSetAttenuation(ADC_11db);

}

double inputVoltage, calVoltage;

void loop() {
  // put your main code here, to run repeatedly:
  calVoltage = analogReadMilliVolts(INPUT_PIN) / 1000.0;

  inputVoltage = analogRead(INPUT_PIN) * 3.3 / (2 << 12);

  Serial.printf("raw= %6.3lf    cali= %6.3lf", inputVoltage, calVoltage);
}

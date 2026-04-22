#include <Arduino.h>
#include <ESP32Servo.h>

#define INPUT1_PIN 35
#define INPUT2_PIN 34
#define INPUT3_PIN 33
#define LED_PIN 27

#define FREQ  50
#define RES  10
#define I1 5
#define I2 2

#define MAXIMUM_NEGATIVE 1000
#define MAXIMUM_POSITIVE 2000

Servo motor;

void setup() {

  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(INPUT3_PIN, INPUT_PULLDOWN);
  Serial.begin(115200);

  while(!Serial);
  delay(1000);

  /*
  ledcSetClockSource(LEDC_USE_RC_FAST_CLK)
  Serial.println(ledcGetClockSource());
  if(!ledcAttach(LED_PIN, FREQ, RES)){
  
  if(!){
    Serial.println("error");
    while(1);
  }
  */

  motor.attach(LED_PIN, MAXIMUM_NEGATIVE, MAXIMUM_POSITIVE);
  while(!digitalRead(INPUT3_PIN));
}

unsigned long output = 0;
bool secondaryOut = false;

void loop() {
  unsigned long input1 = analogReadMilliVolts(INPUT1_PIN);
  unsigned long input2 = analogReadMilliVolts(INPUT2_PIN);
  //Serial.printf("%3ld   %3ld\n", input1, input2);

  float perc1 = (input1 - 142) / (3165.0f - 142.0f);
  float perc2 = (input2 - 142) / (3165.0f - 142.0f);
  //Serial.printf("%3.0f   %3.0f\n", perc1 * 100, perc2 * 100);

  input1 = perc1 * (MAXIMUM_POSITIVE - MAXIMUM_NEGATIVE) + MAXIMUM_NEGATIVE;
  input2 = perc2 * (MAXIMUM_POSITIVE - MAXIMUM_NEGATIVE) + MAXIMUM_NEGATIVE;
  if(digitalRead(INPUT3_PIN)){
    if(secondaryOut){
      secondaryOut = false;
      motor.write(input1);
    }
    else{
      secondaryOut = true;
      motor.write(input2);
    }
  }

  Serial.printf("%4d  %4d\n", input1, input2);
  if(secondaryOut)
    Serial.printf("%4s   /\\\n", "");
  else
    Serial.printf(" /\\\n", "");

  /*
  input1 = perc1 * (I1 - 1);
  input2 = perc2 * ((1 << I2) - 1);
  //Serial.printf("%3ld   %3ld\n", input1, input2);

  for(int i = RES - 1; i >= 0; i--){
    Serial.printf("%d ", !!(output & (1 << i)));
  }
  Serial.printf("= %u (%u)\n", output, ledcRead(LED_PIN));
  Serial.printf("%*s", (I1 - 1 - input1) * I2 * 2, "");
  for(int i = I2 - 1; i >= 0; i--){
    Serial.printf("%d ", !!(input2 & (1 << i)));
  }
  Serial.println();

  if(digitalRead(INPUT3_PIN)){
    output = (output & ~(((1 << I2) - 1) << (input1 * I2))) |  input2 << (input1 * I2);
  }

  //ledcWrite(LED_PIN, output);
  */
  
  delay(250);
}

/*
 * 500 Hz + 10-bit
 * clock
 * max 1022
 * min 292
 *
 * counter
 * max 12
 * min 239
 *
 *
 * 500 Hz + 8-bit
 * clock
 * max 167
 * min 73
 *
 * counter
 * max 12
 * min 59
 *
 *
 * 500 Hz + 16-bit
 * clock
 * max 43776
 * min 18655
 *
 * counter
 * max 190
 * min 15311
 
 
 * 250 Hz + 10-bit 8M 3V3
 * clock
 * max 128
 * min 366
 *
 * counter
 * max 583
 * min 391
 *
 */

#ifndef SIGNAL_CONDITIONING_H
#define SIGNAL_CONDITIONING_H

#include <vector>
#include <stdint.h>
#include <cmath>

class MovingAverage{
  uint16_t movingAverageSamples, ma_index = 0;
  std::vector<float> ma_samples;
  float ma_total = 0;
  bool empty = true;

  float output = 0;

  public:
  MovingAverage(uint16_t samples);

  float update(float input);
  float getValue();

  private:
  float initialize(float input);
  float justUpdate(float input);
};

class LDRNormalizer{
  /*  Um LDR é caracterizado por uma curva `R = b * E^a`, onde `R` é a resistência, `E` é a iluminância sobre o sensor e `a` e `b` são duas constantes
   * próprias de cada LDR.
   * 
   *  Considerando que a ligação será:
   * 
   *  3V3                Vin                  GND
   *   |                  |                    |
   *   +------( LDR )-----+---( resistor r)----+
   *
   * a tensão em `Vin` será de
   *
   *        Vin = 3.3 * r / (LDR + r)
   *
   * por meio da qual, pode se obter a iluminância `E` como
   *
   *        E = beta * (3.3 / Vin - 1) ^ alpha,
   *
   * onde alpha e beta são dois parâmetros dependentes de `a`, `b` e `r`, mais precisamente
   *
   *        alpha = 1 / a
   *        beta = (r / b) ^ alpha.
   *
   *  Para garantir que o ADC do esp32 permita ler todos as possíveis variações na iluminância pelo sol e pelo flash de celular (E <= e ^ 13.6), o valor
   * de r deve ser menor que 450 Ω, idealmente 400 Ω.
  */
  float alpha, beta;

  public:
  LDRNormalizer();
  LDRNormalizer(float alpha, float beta);

  bool calibrate(float refIlluminance1, float refIlluminance2, float rawVoltage1, float rawVoltage2);
  void calibrate(float alpha, float beta);

  float toIlluminance(float rawVoltage);
};

#endif
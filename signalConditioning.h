#ifndef SIGNAL_CONDITIONING_H
#define SIGNAL_CONDITIONING_H

#include <vector>
#include <stdint.h>

class MovingAverage{
  uint16_t movingAverageSamples, ma_index;
  std::vector<double> ma_samples;
  double ma_total;
  bool empty;

  // ponteiro para as entradas e saídas
  double *input;
  double *output;

  public:
  MovingAverage(uint16_t samples, double *input, double *output);

  double update();

  private:
  double initialize();
  double justUpdate();
};

#endif
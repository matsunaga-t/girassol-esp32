#include "signalConditioning.h"

MovingAverage::MovingAverage(uint16_t samples, double *input, double *output)
  : movingAverageSamples(samples),
    ma_index(0),
    ma_samples(std::vector<double>(samples, 0.0)),
    ma_total(0),
    empty(true),
    input(input),
    output(output)   // vírgula solta removida
{}

double MovingAverage::update() {
  if (empty) {
    empty = false;
    return initialize();
  }
  return justUpdate();
}

double MovingAverage::initialize() {
  for (uint16_t i = 0; i < movingAverageSamples; i++) {
    ma_samples[i] = *input;
  }
  ma_total = *input * movingAverageSamples;
  return *output = *input;
}

double MovingAverage::justUpdate() {
  ma_total -= ma_samples[ma_index];
  ma_total += (ma_samples[ma_index] = *input);
  ma_index  = (ma_index + 1) % movingAverageSamples;
  return *output = ma_total / movingAverageSamples;
}

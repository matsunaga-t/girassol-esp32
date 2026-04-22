#include "signalConditioning.h"

MovingAverage::MovingAverage(uint16_t samples):
  movingAverageSamples(samples),
  ma_samples(std::vector<float>(movingAverageSamples))
{
}

float MovingAverage::update(float input){
  if(empty){
    empty = false;
    return initialize(input);
  }
  return justUpdate(input);
}

float MovingAverage::getValue(){
  return this->output;
}

float MovingAverage::initialize(float input){
  for(int16_t i = 0; i < this->movingAverageSamples; i++){
    ma_samples[i] = input;
  }
  this->ma_total = input * this->movingAverageSamples;

  return this->output = input;
}

float MovingAverage::justUpdate(float input){
  this->ma_total -= this->ma_samples[this->ma_index];
  this->ma_total += (this->ma_samples[this->ma_index] = input);
  this->ma_index = (this->ma_index + 1) % this->movingAverageSamples;

  return this->output = this->ma_total / (1.0f * this->movingAverageSamples);
}


LDRNormalizer::LDRNormalizer():
  alpha(0),
  beta(0)
{
}

LDRNormalizer::LDRNormalizer(float alpha, float beta):
  alpha(alpha),
  beta(beta)
{
}

bool LDRNormalizer::calibrate(float refIlluminance1, float refIlluminance2, float rawVoltage1, float rawVoltage2){
  if(rawVoltage1 && rawVoltage2 && refIlluminance1 && refIlluminance2){
    this->alpha = std::logf(refIlluminance2 / refIlluminance1) / std::logf( (3.3f / rawVoltage2 - 1) / (3.3f / rawVoltage1 - 1));
    this->beta = refIlluminance1 / std::powf(3.3f / rawVoltage1 - 1, this->alpha);
    return true;
  }
  return false;
}

void LDRNormalizer::calibrate(float alpha, float beta){
  this->alpha = alpha;
  this->beta = beta;
}

float LDRNormalizer::toIlluminance(float rawVoltage){
  return this->beta * powf(3.3f / rawVoltage - 1, this->alpha);
}


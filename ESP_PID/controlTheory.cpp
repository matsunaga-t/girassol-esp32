#include "controlTheory.h"
#include <esp32-hal.h>

PIDController::PIDController(float Kp, float Ki, float Kd, float *setPoint, float *input, long *output, long lowerBound, long upperBound, long offset):
  Kp(Kp), Ki(Ki), Kd(Kd),
  setPoint(setPoint),
  lowerBound(lowerBound - offset),
  upperBound(upperBound - offset),
  offset(offset),
  oldTime(millis()),
  input(input), output(output)
{
}

void PIDController::reset(){
  this->running = false;
  this->cumulativeError = 0;
  this->oldTime = millis();
}

long PIDController::update(){
  if(!this->running){
    this->running = true;
    this->oldError = this->newError = *this->setPoint - *this->input;
  }
  else{
    this->oldError = this->newError;
    this->newError = *this->setPoint - *this->input;
  }
  newTime = millis();
  return *this->output = pid_controll() + this->offset;
}

void PIDController::setGain(float new_gain, int gain_type){
  switch(gain_type){
    case 0: 
    this->Kp = new_gain;
    break;
    case 1: 
    this->Ki = new_gain;
    break;
    case 2: 
    this->Kd = new_gain;
  }
}

void PIDController::getAllGains(float *Kp, float *Ki, float *Kd){
  *Kp = this->Kp;
  *Ki = this->Ki;
  *Kd = this->Kd;
}

void PIDController::setAllGains(float Kp, float Ki, float Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PIDController::setLimits(int lower, int upper){
  this->lowerBound = lower - this->offset;
  this->upperBound = upper - this->offset;
}

long PIDController::pid_controll(){
  unsigned long timeDelta = (this->newTime = millis()) - this->oldTime;
  this->oldTime = this->newTime;

  float P_term = this->newError * this->Kp;

  #if TRAPEZOIDAL_INTEGRAL_FOR_CUMULATIVE_ERROR
    this->cumulativeError += (this->newError + this->oldTime) / 2 * timeDelta;
  #else
    this->cumulativeError += this->newError * timeDelta;
  #endif
  if (this->cumulativeError > this->upperBound)
    this->cumulativeError = this->upperBound;
  else if (this->cumulativeError < this->lowerBound)
    this->cumulativeError = this->lowerBound;
  
  float I_term = this->cumulativeError * this->Ki;

  float D_term = (this->newError - this->oldError) * this->Kd / timeDelta;

  long result = (long)(P_term + I_term + D_term);
  if (result > this->upperBound)
    result = this->upperBound;
  else if (result < this->lowerBound)
    result = this->lowerBound;
  return result;
}
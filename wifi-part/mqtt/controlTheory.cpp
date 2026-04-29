#include "controlTheory.h"

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double *setPoint, double *input, long *output,
                             long lower_bound, long upper_bound, long offset)
  : Kp(Kp), Ki(Ki), Kd(Kd),
    setPoint(setPoint),
    lower_bound(lower_bound - offset),
    upper_bound(upper_bound - offset),
    offset(offset),
    old_error(0), new_error(0), cumulative_error(0),
    running(false),
    input(input), output(output)   // vírgula solta removida
{}

void PIDController::reset() {
  running = false;
}

// "virtual" removido da definição (só fica no .h)
void PIDController::update(double timeDelta) {
  if (!running) {
    running   = true;
    old_error = new_error = *setPoint - *input;
    cumulative_error = 0;
  } else {
    old_error = new_error;
    new_error = *setPoint - *input;
  }
  *output = pid_controll(timeDelta) + offset;
}

void PIDController::setGain(double new_gain, int gain_type) {
  switch (gain_type) {
    case 0: Kp = new_gain; break;
    case 1: Ki = new_gain; break;
    case 2: Kd = new_gain; break;
  }
}

void PIDController::getAllGains(double *Kp, double *Ki, double *Kd) const {
  *Kp = this->Kp;
  *Ki = this->Ki;
  *Kd = this->Kd;
}

void PIDController::setAllGains(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PIDController::setLimits(long lower, long upper) {
  this->lower_bound = lower - this->offset;
  this->upper_bound = upper - this->offset;
}

long PIDController::pid_controll(double timeDelta) {  // retorno long corrigido
  double P_term = new_error * Kp;

  #if TRAPEZOIDAL_INTEGRAL_FOR_CUMULATIVE_ERROR
    cumulative_error += (new_error + old_error) / 2.0 * timeDelta;
  #else
    cumulative_error += new_error * timeDelta;
  #endif

  if      (cumulative_error > upper_bound) cumulative_error = upper_bound;
  else if (cumulative_error < lower_bound) cumulative_error = lower_bound;

  double I_term = cumulative_error * Ki;
  double D_term = (timeDelta > 0) ? (new_error - old_error) * Kd / timeDelta : 0;

  long result = (long)(P_term + I_term + D_term);
  if      (result > upper_bound) result = upper_bound;
  else if (result < lower_bound) result = lower_bound;
  return result;
}

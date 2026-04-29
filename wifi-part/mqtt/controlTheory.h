#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

#define TRAPEZOIDAL_INTEGRAL_FOR_CUMULATIVE_ERROR 1

class PIDController {
  double Kp, Ki, Kd;
  double *setPoint;
  long lower_bound, upper_bound, offset;

  double old_error, new_error, cumulative_error;
  bool running;

  double *input;
  long   *output;

  public:
  PIDController(double Kp, double Ki, double Kd,
                double *setPoint, double *input, long *output,
                long lower_bound, long upper_bound, long offset);

  void reset();
  virtual void update(double timeDelta);

  void setGain(double new_gain, int gain_type);
  void setAllGains(double Kp, double Ki, double Kd);
  void getAllGains(double *Kp, double *Ki, double *Kd) const;  // const corrigido
  void setLimits(long lower, long upper);

  private:
  long pid_controll(double timeDelta);  // retorno long corrigido
};

#endif

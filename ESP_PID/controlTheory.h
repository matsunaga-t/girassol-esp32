#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

#define TRAPEZOIDAL_INTEGRAL_FOR_CUMULATIVE_ERROR 1

class PIDController{
  // parâmetros do PID
  float Kp, Ki, Kd;
  float *setPoint;
  long lowerBound, upperBound, offset;

  // variáveis para o sistema
  float oldError = 0, newError = 0, cumulativeError = 0;
  unsigned long oldTime, newTime = 0;
  bool running = false;

  // ponteiros para entrada e saída
  float *input;
  long *output;

  public:
  PIDController(float Kp, float Ki, float Kd, float *setPoint, float *input, long *output, long lowerBound, long upperBound, long offset);

  void reset();
  long update();

  void setGain(float new_gain, int gain_type);
  void setAllGains(float Kp, float Ki, float Kd);

  void getAllGains(float *Kp, float *Ki, float *Kd);

  void setLimits(int lower, int upper);

  private:
  long pid_controll();
};

#endif
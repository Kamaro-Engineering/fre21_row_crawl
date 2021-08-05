#include <fre_row_navigation/pid.h>

double PID::calculate(double error) {
  integrator += error;
  double result = p * error + i * integrator + d * (error - lastError);
  lastError = error;

  return result;
}

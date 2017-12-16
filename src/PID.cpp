#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  prev_error = -0.7598;

  d_buffer = new double[1];

  d_sum = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = cte - prev_error;

  d_sum -= d_buffer[0];
  d_buffer[0] = d_error;
  d_sum += d_buffer[0];

  d_error = d_sum;

  prev_error = cte;
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}


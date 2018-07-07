#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double error) {
  if(!init_d){
    d_error = error;
    init_d = true;
    prev_error = error;
  }else{
    d_error = error - prev_error;
    prev_error = error;
  }
  p_error = error;
  i_error +=error;
}

double PID::TotalError() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	Kp = Ki = Kd = 0;
	p_error = i_error = d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = i_error = d_error = 0;
}

void PID::UpdateError(double cte) {
	d_error  = (cte - p_error);
	p_error  = cte;
	i_error += cte;
}

double PID::TotalError() {
	return Kp * p_error + Ki * i_error + Kd * d_error;
}


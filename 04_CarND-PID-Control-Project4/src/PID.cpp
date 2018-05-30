#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Kp is used to adjust the error to be smaller
    // If vehicle turns a small angle, increase KP. If vehicle overshoots immediately, decrease KP
    this->Kp = Kp;
    // Ki is used to remove the systematic bias  
    // If vehicle is always off the center of the lane, increase Ki
    this->Ki = Ki;
    // Kd is used to remove the bounding effect from propotional control
    // If vehicle keep bounding, increase Kd
    this->Kd = Kd;
    this->cte_initialized = false;
    this->int_cte = 0.0;
}

void PID::UpdateError(double cte) {
	if (cte_initialized == false) {
		prev_cte = cte;
		cte_initialized = true;
	}
	cur_cte = cte;
    diff_cte = cur_cte - prev_cte;
	int_cte += cte;
	prev_cte = cte;
}

double PID::TotalError() {
    double res = -Kp * cur_cte - Ki * int_cte - Kd * diff_cte;
	res = res < -1 ? -1 : res;
	res = res > 1 ? 1 : res;

	return res;
}


#include <iostream>
#include <stdlib.h>  
#include <cmath>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;  // this p_error is previous cte
	p_error = cte;
	i_error += cte;
	err += cte * cte;
}

double PID::TotalError(){
	numSteps = 100;
	cout << " err = " << err << " numSteps = " << numSteps << "\n";
	t_err = err/numSteps;
	return t_err;
}

double PID::SteeringAngle(){
	double steeringAng;
	steeringAng = Kp*p_error + Ki*i_error + Kd*d_error;

	if (steeringAng < -1){
		steeringAng = -1;
	} else if (steeringAng > 1){
		steeringAng = 1;
	}
	return steeringAng;
}

void PID::Twiddle2(double cte){
	double dp_p = 0.001;
  double dp_i = 0.001;
  double dp_d = 0.001;
  p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	double t_err = TotalError();

	double best_err = t_err;

	int tot_num = 30;  // 20
	int n = 0;
	double tol = 0.001; // 0.002

	double strang;
	Init(-0.1, 0.01, -0.1);

	double dp_sum = dp_p+dp_i+dp_d;

	while ((fabs(dp_sum) > tol) && (n < tot_num)){
		//cout << "n = " << n << "\n";
			UpdateError(cte);
			t_err = TotalError();
			//cout << "best_err = " << best_err << " t_err = " << t_err << "\n";
				Kp -= dp_p;
				Ki -= dp_i;
				Kd -= dp_d;

			if (t_err < best_err){
				best_err = t_err;
					dp_p *= 1.1;
					dp_i *= 1.1;
					dp_d *= 1.1;
			} else {
					Kp -= 2.0 * dp_p;
					Ki -= 2.0 * dp_i;
					Kd -= 2.0 * dp_d;

				UpdateError(cte);
				t_err = TotalError();

				if (t_err < best_err){
					best_err = t_err;
 						dp_p *= 1.1;
 						dp_i *= 1.1;
 						dp_d *= 1.1;
				} else{
						Kp += dp_p;
						dp_p *= 0.9;
						Ki += dp_i;
						dp_i *= 0.9;
						Kd += dp_d;
						dp_d *= 0.9;	
				} // else close
			}

	  	strang = Kp*p_error + Ki*i_error + Kd*d_error;

	  	//cout << "p_error = " << p_error << " i_error = " << i_error << " d_error = " << d_error << "\n";
	  	//cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << "\n";
 	  	//cout << "new steering angle = " << strang << "\n";
 	  	dp_sum = dp_p+dp_i+dp_d;
		n += 1;

	}
	cout << "final: Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << "\n";

}



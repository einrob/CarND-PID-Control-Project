#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool show_output_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

	p_error = 0;
	i_error = 0;
	d_error = 0;

	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	total_error = 0;
	sum_cte = 0;
	cte_prev = 0;
  
  	show_output = show_output_;


}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

	sum_cte += cte;
	sum_cte = std::max(-1000.0, std::min(sum_cte, 1000.0)); // https://stackoverflow.com/questions/9323903/most-efficient-elegant-way-to-clip-a-number clipping a variable

	p_error = cte;
	i_error = sum_cte;
	d_error = ((cte-cte_prev)/0.02);  /// divided by time_span !?

	cte_prev = cte;

	if(show_output == true)
    {
      	std::cout << "p_error: " << p_error << "      i_error: " << i_error << "          d_error: " << d_error << std::endl;
    }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

	total_error = Kp*p_error + Kp*d_error + Ki*i_error;


  return total_error;  // TODO: Add your total error calc here!
}

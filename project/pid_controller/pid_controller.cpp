/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   for(int i = 0; i < 3; i++)
   {
      pid_errors[i] = 0;
   }
   pid_coeffs[0] = Kpi;
   pid_coeffs[1] = Kii;
   pid_coeffs[2] = Kdi;
   pid_output_max = output_lim_maxi;
   pid_output_min = output_lim_mini;

}


void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   


   // Update differential term
   if(delta_time > 0){
      pid_errors[2] = (cte - pid_errors[0])/delta_time;
   }
   else {
      pid_errors[2] = 0;
   }
   // Propotional term
   pid_errors[0] = cte;
   // Intergral term
   pid_errors[1] += cte * delta_time;
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = 0.0;
    // Sum up propotional, integral and differential terms
    for(int i = 0; i < 3; i++){
       control -= pid_coeffs[i] * pid_errors[i];
       std::cout << " PID " << i << " " << pid_errors[i];
    }
    std::cout << " Output " << control << std::endl;
    // Clip by max and min output control values
    control = min(pid_output_max, control);
    control = max(pid_output_min, control);
    std::cout << " Output x" << control << std::endl;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}

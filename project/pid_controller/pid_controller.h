/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * Create the PID class
   **/

    /*
    * Errors
    */
    double pid_errors[3];
    double prev_error;

    /*
    * Coefficients
    */
    double pid_coeffs[3];

    /*
    * Output limits
    */
    double pid_output_max;
    double pid_output_min;
    double pid_min_integral;
    double pid_max_integral;
  
    /*
    * Delta time
    */
    double delta_time;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H



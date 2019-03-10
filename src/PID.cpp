#include "PID.h"

#include <iostream>
#include <math.h>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  i_error = 0;
  previous_cte = 0;
  counter = 0;
  total_error = 0;
  error = 0;

  std::cout << "p: " << Kp << " i: " << Ki << " d: " << Kd  << std::endl;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  p_error = cte;

  d_error = cte - previous_cte;
  previous_cte = cte;

  i_error += cte;

  total_error += pow(cte, 2);
  counter++;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}

double PID::SteeringOutput(double max_steering_angle) {
    double output = TotalError();

    if(output > max_steering_angle)
        output = max_steering_angle;
    if(output < -max_steering_angle)
        output = -max_steering_angle;

    return output;
}

double PID::ThrottleOutput(double min_throttle, double max_throttle) {
  /**
   * Calculate throttle output
   */
  double output = fabs(TotalError());
  double throttle_limit = 1;

  // Cap throttle output value at throttle limit
  if(output > throttle_limit) {
    output = throttle_limit;
  }

  /**
   * Using steering value as the CTE for throttling.
   * Throttle output is inversely proportional to the steering value.
   * A small steering value (i.e going straight) will mean higher throttle.
   * Conversely, a large steering value (going around a corner) will mean lower throttle.
   */
  output = 1 - output;

  // Apply throttle scaling factor according to the max throttle
  output *= (max_throttle - min_throttle);  // Cap at 30
  output += min_throttle;  // Add minimum throttle

  return output;
}

double PID::AverageSquaredError() {
  return total_error/counter;
}

void PID::Twiddle(double tolerance) {
  if(twiddle) {
    std::cout << "counter: " << counter << " coeff: " << coeff << " substate: " << twiddle_state << " batch " << (counter % batch_size) << std::endl;
    std::cout << "p: " << Kp << " dp_p: " << dp_p
              << " d: " << Kd << " dp_d: " << dp_d
              << " i: " << Ki << " dp_i: " << dp_i
              << std::endl;
    std::cout << "best error: " << best_error << " error: " << error/((counter % batch_size) +1)<< std::endl;
    std::cout << "Best p: " << best_p << " Best i: " << best_i << " Best d: " << best_d << std::endl;

    // Initialize first best error
    if (counter == batch_size) {
      best_error = total_error / batch_size;
      best_p = Kp;
      best_i = Ki;
      best_d = Kd;
    }

    if (counter >= batch_size) {
      // Increment coefficient
      if (twiddle_state == 0) {
        IncrementCoefficient(coeff);
        twiddle_state = 1;  // Next measure increment
      } else if (twiddle_state == 1 || twiddle_state == 3) {
        // Measuring
        error += pow(p_error, 2);

        if ((counter % batch_size) == 0) {
          twiddle_state++;
        }
      } else if (twiddle_state == 2) {
        // Compare error measurement from increment coefficient
        double err = error / batch_size;
        error = 0;  // Reset error
        if (err < best_error) {
          best_error = err;
          best_p = Kp;
          best_i = Ki;
          best_d = Kd;

          // Increment coefficient modifier
          IncrementCoefficientModifier(coeff);

          // Set to next coefficient to apply twiddle
          NextCoefficient();

          // Return to state 0 (increment coefficient)
          twiddle_state = 0;

          // Check if coefficient modifier hit tolerance limit
          TolerenceCheck(tolerance);

        } else {
          // Best error not found, decrement coefficient
          DecrementCoefficient(coeff);

          // Next measure error for decreased coefficient
          twiddle_state = 3;
        }
      } else if (twiddle_state == 4) {
        // Compare error measurement from decreased coefficient
        double err = error / batch_size;
        error = 0;  // Reset error
        if (err < best_error) {
          best_error = err;
          best_p = Kp;
          best_i = Ki;
          best_d = Kd;
          // Increment coefficient modifier
          IncrementCoefficientModifier(coeff);
        } else {
          // Best error not found, decrement coefficient modifier
          IncrementCoefficient(coeff);
          DecrementCoefficientModifier(coeff);
        }

        // Set to next coefficient to apply twiddle
        NextCoefficient();
        twiddle_state = 0;

        // Check if coefficient modifier hit tolerance limit
        TolerenceCheck(tolerance);
      }
    }
  }
}


void PID::IncrementCoefficientModifier(int coeff) {
  switch (coeff) {
    case 0:
      dp_p *= 1.1;
      break;
    case 1:
      dp_d *= 1.1;
      break;
    case 2:
      dp_i *= 1.1;
      break;
    default:
      break;
  }
}

void PID::DecrementCoefficientModifier(int coeff) {
  switch (coeff) {
    case 0:
      dp_p *= 0.9;
      break;
    case 1:
      dp_d *= 0.9;
      break;
    case 2:
      dp_i *= 0.9;
      break;
    default:
      break;
  }
}

void PID::IncrementCoefficient(int coeff) {
  switch (coeff) {
    case 0:
      Kp += dp_p;
      break;
    case 1:
      Kd += dp_d;
      break;
    case 2:
      Ki += dp_i;
      break;
    default:
      break;
  }
}

void PID::DecrementCoefficient(int coeff) {
  switch (coeff) {
    case 0:
      Kp -= 2 * dp_p;
      break;
    case 1:
      Kd -= 2 * dp_d;
      break;
    case 2:
      Ki -= 2 * dp_i;
      break;
    default:
      break;
  }
}

void PID::NextCoefficient() {
  coeff = (coeff + 1) % 3;
}

void PID::TolerenceCheck(double tolerance) {
  if((dp_d + dp_p + dp_i) <= tolerance) { twiddle = false; }
}
#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Output average squared error per cycle
   * @return
   */
  double AverageSquaredError();

  double SteeringOutput(double max_steering_angle = 1);

  /**
   * Output throttle based on PID control and scaled the throttled according to the set
   * minimum and maximum throttle.
   * @param min_throttle
   * @param max_throttle
   * @return
   */
  double ThrottleOutput(double min_throttle = 0.1, double max_throttle = 0.3);

  void Twiddle(double tolerance = 0.2);

  void IncrementCoefficientModifier(int coeff);
  void DecrementCoefficientModifier(int coeff);

  void IncrementCoefficient(int coeff);
  void DecrementCoefficient(int coeff);

  void TolerenceCheck(double tolerance = 0.2);

  void NextCoefficient();

  int counter;  // Track count

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double previous_cte;

  double total_error;
  double best_error;
  double best_p;
  double best_i;
  double best_d;

  /**
   * Twiddle batch size
   */
  int batch_size = 50;

  /**
   * Current error count within the twiddle cycle
   */
  double error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Coefficient modifiers used by twiddle
   */
  double dp_p = 0.1;
  double dp_d = 1;
  double dp_i = 0.0001;

  /**
   * Track which coefficient is being updated
   * 0 - P
   * 1 - D
   * 2 - I
   */
  int coeff = 0;

  /**
   * Track the state of the twiddle algorithm within the batch process
   * 0 - Increment coefficient
   * 1 - Measure error for the increase coefficient
   * 2 - Compare increased coefficient against best error and either go back to state 0 or decrease coefficient.
   * 3 - Measure error for the decreased coefficient
   * 4 - Compare decreased coefficient against best error
   */
  int twiddle_state = 0; // 0 Increment coeef, 1 measure increased coeef, 2 compare increased coeff,  3 measure decreased coeef, 4 compare decrease coef

  bool twiddle = true;

};

#endif  // PID_H
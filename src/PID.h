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

  double AverageSquaredError();

  double SteeringOutput();
  double ThrottleOutput(double max_throttle = 0.3, double min_throttle = 0.1);

  void IncrementCoefficientModifier(int coeff);
  void DecrementCoefficientModifier(int coeff);

  void IncrementCoefficient(int coeff);
  void DecrementCoefficient(int coeff);

  void TolerenceCheck(double tolerance = 0.2);

  void NextCoefficient();

  void Twiddle(double tolerance = 0.2);

  int counter;  // Track count

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double previous_cte;

  int batch_size = 50;
  double total_error;
  double best_error;

  double error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  double dp_p = 0.1;
  double dp_d = 1;
  double dp_i = 0.0001;

  int coeff = 0; // 0 - P, 1 - D, 2 - I
  int twiddle_state = 0; // 0 Increment coeef, 1 measure increased coeef, 2 compare increased coeff,  3 measure decreased coeef, 4 compare decrease coef

  bool twiddle = true;

};

#endif  // PID_H
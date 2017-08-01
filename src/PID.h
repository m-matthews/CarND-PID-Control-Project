#ifndef PID_H
#define PID_H

enum TwiddleTest { up, down };

class PID {
public:
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Initialize Twiddle.
  */
  void Twiddle(double Tp, double Ti, double Td, unsigned int missSteps, unsigned int usedSteps);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Determine if the Twiddle has reset in the last step.
  */
  bool TwiddleReset();

private:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients (Elements 0=P, 1=I, 2=D)
  */
  double p[3];

  /*
  * Twiddle Coefficients (Elements 0=P, 1=I, 2=D)
  */
  double dp[3];

  /*
  * PID and Twiddle Calculation Values
  */
  double prev_cte;                // Previous CTE.
  double integral_cte;            // CTE Integral.
  double total_cte;               // Sum of all CTEs.
  long int count_cte;             // Count of all CTEs.

  bool twiddle;                   // Twiddle enabled flag.
  double twiddle_error;           // Current iteration error for Twiddle.
  double twiddle_besterror;       // Best error for Twiddle.
  unsigned int twiddle_iters;     // Twiddle iteration counter.

  unsigned int twiddleStep;       // Current Step counter for this Twiddle iteration.
  unsigned int twiddleMissSteps;  // Number of Steps to ignore at the start of a Twiddle iteration.
  unsigned int twiddleTotalSteps; // Total Number of Steps to use in a Twiddle iteration.
  unsigned int twiddleParam;      // Parameter (0-2) Twiddled in this iteration.

  bool firstTwiddle;              // Is this the first Twiddle iteration?
  TwiddleTest twiddleTest;        // Current Twiddle direction.
  bool twiddleReset;              // Flag to indicate the Simulator needs a reset for the next iteration.
};

#endif /* PID_H */

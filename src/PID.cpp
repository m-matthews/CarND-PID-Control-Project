#include <iostream>
#include <limits>
#include <math.h>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  twiddle = false;

  prev_cte = 0.0;
  integral_cte = 0.0;

  total_cte = 0.0;
  count_cte = 0;
}

void PID::Twiddle(double Tp, double Ti, double Td, unsigned int missSteps, unsigned int usedSteps) {
  // Initialise Twiddle state.
  twiddle = true;
  firstTwiddle = true;
  twiddleReset = false;
  twiddle_iters = 0;

  dp[0] = Tp;
  dp[1] = Ti;
  dp[2] = Td;

  twiddleStep = 0;
  twiddleMissSteps = missSteps;
  twiddleTotalSteps = missSteps + usedSteps;
  twiddleParam = 0;
  twiddleTest = up;

  twiddle_error = 0.0;
  twiddle_besterror = 0.0;
}

void PID::UpdateError(double cte) {
  // PID error update.
  p_error = p[0] * cte;
  i_error = p[1] * integral_cte;
  d_error = p[2] * (cte - prev_cte);

  prev_cte = cte;
  integral_cte += cte;

  total_cte += fabs(cte);
  count_cte++;

  // Twiddle calculation.
  if(twiddle)
  {
    bool twiddleIterationDone = false;

    if(twiddleStep == 0)
      cout << "Starting twiddle #" << twiddle_iters++ << ": (" << p[0] << ", " << p[1] << ", " << p[2] << ") ... " << endl;

    // Early detection for any twiddle variations that may leave the track.
    if(cte>3.5)
    {
      twiddleStep = twiddleTotalSteps;
      twiddle_error = std::numeric_limits<double>::max();
    }

    if(++twiddleStep > twiddleMissSteps)
      twiddle_error += cte*cte;

    // Check for end of Twiddle Test.
    if(twiddleStep > twiddleTotalSteps)
    {
      if(firstTwiddle)
      {
        firstTwiddle = false;
        twiddle_besterror = twiddle_error;
        twiddleTest = up;
        p[twiddleParam] += dp[twiddleParam];
      }
      else
      {
        if(twiddleTest == up)
        {
          if (twiddle_error < twiddle_besterror)
          {
            twiddle_besterror = twiddle_error;
            dp[twiddleParam] *= 1.1;
            twiddleIterationDone = true;
          }
          else
          {
            twiddleTest = down;
            p[twiddleParam] -= 2.0 * dp[twiddleParam];
          }
        }
        else if(twiddleTest == down)
        {
          if (twiddle_error < twiddle_besterror)
          {
            twiddle_besterror = twiddle_error;
            dp[twiddleParam] *= 1.1;
            twiddleIterationDone = true;
          }
          else
          {
            p[twiddleParam] += dp[twiddleParam];
            dp[twiddleParam] *= 0.9;
            twiddleIterationDone = true;
          }
        }
      }
      cout << "Twiddle result: error = " << twiddle_error << " (best = " << twiddle_besterror << ")" << endl;

      if(twiddleIterationDone)
      {
        // Increment to the next Twiddle Parameter.
        twiddleParam = (twiddleParam+1) % 3;
        twiddleTest = up;
        p[twiddleParam] += dp[twiddleParam];
      }

      if(dp[0] + dp[1] + dp[2] < 0.00001)
      {
        cout << "Twiddle solved!" << endl;
        twiddle = false;
      }
      else
      {
        twiddleReset = true;
        twiddle_error = 0.0;
        twiddleStep = 0;
      }
    }
  }
  else if (count_cte%100 == 0)
  {
    cout << "Average CTE = " << total_cte/(double)count_cte << endl;
  }
}

double PID::TotalError() {
  double steer = -p_error - d_error - i_error;

  return steer;
}

bool PID::TwiddleReset() {
  if(twiddle && twiddleReset)
  {
    twiddleReset = false;
    twiddle_error = 0.0;
    twiddleStep = 0;
    return true;
  }

  return false;
}

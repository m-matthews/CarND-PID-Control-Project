# PID Control Project


## PID Controller Development

The files [PID.h](src/PID.h) and [PID.cpp](src/PID.cpp) contain the standard PID controller logic based on the lessons for Term 2, Lesson 16: PID Control, including a Twiddle optimiser.  The use of the Twiddle optimiser is selected with `#DEFINE TWIDDLE` at the top of the [main.cpp](src/main.cpp) program.


## Speed Selection

The code initially provided included a constant throttle value which resulted in the vehicle leaving the track quickly.  A simple `P` Controller was implemented for the throttle / speed which settles between 28mph and 29mph.


## PID Parameter Selection

The CTE measurement used for selecting the optimal values was an Average CTE reported after each 100 steps in [PID.cpp](src/PID.cpp):

```cpp
  else if (count_cte%100 == 0)
  {
    cout << "Average CTE = " << total_cte/(double)count_cte << endl;
  }
```

The selection of the `P`, `I` and `D` parameters were based on reading George Gillard's document [pid_control_document.pdf](http://georgegillard.com/documents).

The sequence of selection for the `P`, `I` and `D` are shown in the commentary at the top of [main.cpp](src/main.cpp) and repeated here for information.

```cpp
  // PID settings based on recommendations from 'pid_control_document.pdf' at http://georgegillard.com/documents.
  pid.Init(0.10, 0.0000, 0.00); // 1) Long increasing oscillations - does not make it to first corner.
  pid.Init(0.20, 0.0000, 0.00); // 2) Faster oscillations leaving track earlier.
  pid.Init(0.40, 0.0000, 0.00); // 3) Even faster oscillations leaving track earlier - acrobatic style!
  pid.Init(0.10, 0.0000, 1.00); // 4) Low P from above, plus D - first complete track circuit!
  pid.Init(0.10, 0.0000, 2.00); // 5) Increased D - improved Average CTE from step above!
  pid.Init(0.10, 0.0000, 3.00); // 6) Increased D - slightly better Average CTE from step above!
  pid.Init(0.10, 0.0010, 3.00); // 7) Add I - more 'jumpy' but Average CTE is improved.
  pid.Init(0.10, 0.0030, 3.00); // 8) Increase I - Average CTE worse.
  pid.Init(0.10, 0.0020, 3.00); // 9) Decrease I - Average CTE closer to 7).
  pid.Init(0.10, 0.0015, 3.00); // 10) Average I from 7) and 9) - Average CTE closer to 7).
```

Modification of the `P` parameter by itself could not allow the vehicle to navigate around the track.  Introducing `D` provided an immediate improvement and a suitable starting value was found.  Based on the output of Twiddle a review of `P` could have been taken again at this point.  Introduction of the `I` values produced a noticeable improvement in the Average CTE value, however this improvement was not easy to see visually.

The best selection from the above process was in step 7).  These values were then used with the Twiddle optimiser enabled to try to find a more optimal solution.

```cpp
  // Final selection with best Average CTE across the track used for Twiddle input.
  pid.Init(0.10, 0.0010, 3.00);

#ifdef TWIDDLE
  pid.Twiddle(0.05, 0.0001, 0.3, 150, 1650);
#endif
```

The output of the Twiddle process is visible in the [TwiddleOutput.txt](src/TwiddleOutput.txt) file.  The final parameters are then used for the final assignment of PID in the non-Twiddle execution of the Simulator.

```cpp
  // Final Twiddle parameters.
  pid.Init(0.458088, 0.00119765, 4.65089);
```

The vehicle can now successfully navigate around the track.

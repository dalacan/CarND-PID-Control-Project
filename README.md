# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)
[cte_vs_p_fixed_throttle_15]: ./results/cte_vs_p_fixed_throttle_15.png "CTE vs. P - Fixed throttle 15mph"
[cte_vs_p_fixed_throttle_30]: ./results/cte_vs_p_fixed_throttle_30.png "CTE vs. P - Fixed throttle 30mph"
[cte_vs_p_fixed_throttle_60]: ./results/cte_vs_p_fixed_throttle_60.png "CTE vs. P - Fixed throttle 60mph"
[cte_vs_p_fixed_throttle_90]: ./results/cte_vs_p_fixed_throttle_90.png "CTE vs. P - Fixed throttle 90mph"
[cte_vs_p_variable_throttle_max_30]: ./results/cte_vs_p_variable_throttle_max_30.png "CTE vs. P - Variable throttle 30mph"

[cte_vs_small_d_fixed_thottle_30]: ./results/cte_vs_small_d_fixed_thottle_30.png "CTE vs. small D - Fixed throttle 30mph"
[cte_vs_small_d_fixed_thottle_30_zoomed]: ./results/cte_vs_small_d_fixed_thottle_30_zoomed.png "CTE vs. small D - Fixed throttle 30mph (zoomed)"
[cte_vs_mid_d_fixed_thottle_30]: ./results/cte_vs_mid_d_fixed_thottle_30.png "CTE vs. mid D - Fixed throttle 30mph"
[cte_vs_mid_d_fixed_thottle_30_zoomed]: ./results/cte_vs_mid_d_fixed_thottle_30_zoomed.png "CTE vs. mid D - Fixed throttle 30mph (zoomed)"
[cte_vs_large_d_fixed_thottle_30]: ./results/cte_vs_large_d_fixed_thottle_30.png "CTE vs. large D - Fixed throttle 30mph"
[cte_vs_large_d_fixed_thottle_30_zoomed]: ./results/cte_vs_large_d_fixed_thottle_30_zoomed.png "CTE vs. large D - Fixed throttle 30mph (zoomed)"

[cte_vs_i_fixed_throttle_15]: ./results/cte_vs_i_fixed_throttle_15.png "CTE vs. I - Fixed throttle 15mph"
[cte_vs_small_i_fixed_throttle_15]: ./results/cte_vs_small_i_fixed_throttle_15.png "CTE vs. small I - Fixed throttle 15mph"

[cte_vs_i_fixed_d_4_throttle_30]: ./results/cte_vs_i_fixed_d_4_throttle_30.png "CTE vs. I - Fixed throttle 30mph"


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Run Instructions

The application has been pre-built with preset PID values for both the steering and throttle control. To run the application with the preset values run `./pid`.

Additionally, the application features:
- Customizable PID values for both steering and throttle. 
- Automatic PID hyperparameter tweaking through Twiddle
- Customizing minimum and maximum throttle

This can be set by setting the relevant inputs to the `./pid` applications. See usage below:

```
Usage

 pid [options]

Options
-s <p> <i> <d>                                       = Set Steering PID values (-s 0.3 0.0001 3)
-t <p> <i> <d>                                       = Set Throttle PID values (-t 0.9 0 2)
-ws <tolerance> <batch_size>                         = Enable twiddle for steering and set tolerance
                                                       and batch size (-ws 0.2 50)
-ws <tolerance> <batch_size> <p_mod> <i_mod> <d_mod> = Enable twiddle for steering and set tolerance,
                                                       batch size and pid modifiers (-ws 0.2 50 0.1 0.0001 1)
-wt <tolerance> <batch_size>                         = Enable twiddle for throttle and set tolerance
                                                       and batch size (-wt 0.2 50)
-wt <tolerance> <batch_size> <p_mod> <i_mod> <d_mod> = Enable twiddle for throttle and set tolerance,
                                                       batch size and pid modifiers (-wt 0.2 50 0.1 0.0001 1)
-l <min_throttle> <max_throttle>                     = Set minimum and maximum throttle (-l 0.1 0.3)
-d                                                   = Debug output enabled
-h                                                   = Displays this help menu
```

## Effects of PID hyperparameters
This section explores the effects of the PID hyperparameters by observing the cross track error (CTE) over time. 
These test were ran in the first section of the simulator track whereby the vehicle has a gentle left turn trajectory at
the beginning eventuating into a sharper left turn.

### P value
To explore the effects of the P parameter, multiple tests were setup with different P values and the other hyperparameters (I and D) set to 0.
In addition to varying the P values, each set of test was conducted with specific throttle setting to explore the effects of the throttle on the P parameter.

#### Test 1.1 - P values with fixed throttle @ 15mph
Test 1.1 was setup with the following control values:
- Fixed throttle of 15mph
- P values of 0.1, 0.2, 0.3, 0.4, 0.5, 1, 2 and 3
- I and D values of 0

As depicted in the graph below, a higher P value shows the CTE reducing at a faster rate and with a smaller oscillation period.

![alt text][cte_vs_p_fixed_throttle_15]

#### Test 1.2 - P values with fixed throttle @ 30mph
Test 1.2 was setup with the following control values:
- Fixed throttle of 30mph
- P values of 0.1, 0.2, 0.3, 0.4, 0.5, 1 and 2
- I and D values of 0

Compared to the test 1.1, at a higher velocity, it is observed that the CTE reduces at a faster rate. However, over time larger 
values of P (1 and 2) tends to result in larger oscillations and higher P values resulted in the vehicle going off track.

![alt text][cte_vs_p_fixed_throttle_30]

#### Test 1.3 - P values with fixed throttle @ 60mph
Test 1.3 was setup with the following control values:
- Fixed throttle of 60mph
- P values of 0.05 and 0.1
- I and D values of 0

When increasing the speed to 60mph, higher values of Ps were unsustainable and resulted in the vehicle quickly going off track.
As depicted in the graph below, at 0.1, the vehicle eventually when off track (CTE > 5).

![alt text][cte_vs_p_fixed_throttle_60]

#### Test 1.4 - P values with fixed throttle @ 90mph
Test 1.4 was setup with the following control values:
- Fixed throttle of 90mph
- P values of 0.05 and 0.1
- I and D values of 0

This test was conducted at a higher velocity and a similar result to test 1.3 was observed.

![alt text][cte_vs_p_fixed_throttle_90]

#### Test 1.5 - P values with variable throttle: Maximum 30mph
Test 1.4 was setup with the following control values:
- Variable throttle of 10 to 30mph
- P values of 0.1, 0.2, 0.3, 0.4, 0.5, 1 and 2
- I and D values of 0

This test was conducted with a variable throttle whereby the throttle is controlled via a throttle PID control. 
The steering value is fed into the throttle PID control which returns an inversely proportional throttle value 
with respect to the steering value. For example, a large steering value results in a smaller throttle and vice versa.

When adding a variable throttle and comparing to test 1.2 (which had a fixed throttle of 30mph), it is observed 
that at higher P values, the oscillation has smaller peaks and the vehicle is able to stay on track.

![alt text][cte_vs_p_variable_throttle_max_30]

#### Observations for P tests
Based on the above tests, the following key observations were made:
- A higher P value results in the CTE quickly reducing and oscillating at a higher rate around the x-axis
- At higher velocity, a higher P value results in the CTE oscillation peaks quickly increasing over time, thus resulting 
in an unsustainable trajectory
- Lower P value yield a stable trajectory for small curves. However on sharper curves, the vehicle was not able to turn
fast enough, resulting in under-steering.
- With a variable throttle, a higher P value can be sustained.

### D Value
To explore the effects of the D parameter, test was setup with varying I values ranging from:
- Small I values
- Medium I values
- Large I values

#### Test 2.1 - Small D values
Test 2.1 was setup with the following control values:
- Fixed throttle of 30mph
- Fixed P value of 0.3 
- D values set to 0
- I values of 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 5

As depicted in the graph below, it is observed that as I increases, the oscillation peak decreases.

![alt_text][cte_vs_small_d_fixed_thottle_30]

In the magnified version of the graph, the overshoot trajectory of the vehicle around the x-axis gets smaller as 
I in increased. Thus resulting in a smoother trajectory towards the x-axis. 

![alt_text][cte_vs_small_d_fixed_thottle_30_zoomed]

#### Test 2.2 - Medium D values
Test 2.1 was setup with the following control values:
- Fixed throttle of 30mph
- Fixed P value of 0.3 
- D values set to 0
- I values of 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5

When testing slightly larger values of I, similar a similar behavior to test 2.1 is observed. However, the oscillation
is does not oscillate around the x-axis but instead above it, indicating some sort of over-steer. Additionally, the peaks 
between each oscillation reduces as I increases.

![alt_text][cte_vs_mid_d_fixed_thottle_30]

In the magnified version of the graph below, it can be observed that a smooth trajectory to the x-axis without 
overshooting could be achieved with a D value around 4.5.

![alt_text][cte_vs_mid_d_fixed_thottle_30_zoomed]

#### Test 2.3 - Large D values
Test 2.1 was setup with the following control values:
- Fixed throttle of 30mph
- Fixed P value of 0.3 
- D values set to 0
- I values of 7, 7.5, 8, 8.5, 9, 10, 12.5, 15, 20

When testing with large values of I, similar to test 2.2, the oscillation offset increases with larger values of I.

![alt_text][cte_vs_large_d_fixed_thottle_30]

The magnified version of the graph below shows that the CTE never reduces to 0.

![alt_text][cte_vs_large_d_fixed_thottle_30_zoomed]

#### Observations for D tests
The following observations can be made from the above tests:
- A higher D value smooths the trajectory of the vehicle steering around a curve.
- However, a higher D value tends to result in more over-steer and steering adjustments.

### I Value
To explore the effect of the I parameter, two tests were setup, each with varying I values. 

#### Test 3.1 - I values with D=0, fixed throttle @ 15mph
This test was setup with the following control values:
- Fixed P values of 0.3
- Fixed D value of 0
- Fixed throttle of 15mph
- I values of 0, 0.0001, 0.001, 0.01, 0.1 and 1

Initially a throttle of 30mph was used, however at higher I values, the vehicle in the simulator quickly went off track 
before the first counter-steer. Thus a lower throttle of 15mph was chosen. 

As shown in the graph below, the effect of I causes the CTE to 'shift' to the x-axis.
Also, at higher I values (0.1 and 1), the CTE shifted significantly more toward the x-axis and had to aggressively 
counter-steer to correct itself. This resulted in the vehicle eventually going off track.

Also, when I is 0, it is observed that the vehicle does not have a steering bias as the CTE oscillates around the
x-axis.

![alt_text][cte_vs_i_fixed_throttle_15]

Looking closer at I values between 0 and 0.001, at very small I values (0.0001), the shift of the CTE trajectory towards 
the x-axis is very minimal.

![alt_text][cte_vs_small_i_fixed_throttle_15]

#### Test 3.2 - I values with D=4, fixed throttle @ 30mph
This test was setup with the following control values:
- Fixed P values of 0.3
- Fixed D value of 4
- Fixed throttle of 30mph

With the D value set to 4, the trajectory is smoother toward the x-axis. Similar to test 3.1, the curve moves downwards 
towards the x-axis as i is increased.

![alt_text][cte_vs_i_fixed_d_4_throttle_30]

#### Observations for I tests
The following observation can be made for the above tests:
- The greater the I value, the further 'down' the curve is shifted toward the x-axis.
- The vehicle does not have a steering bias. 
- However, test 2.x, a higher D value seem to create an artificial steering bias. Additionally, a driving the vehicle 
around the bends at higher velocity could also cause cause the car to under-steer. And it might be possible to use the 
I coefficient to compensate for this steering biases.

## PID hyperparameter tuning & optimization
In order to achieve get PID steering control for the vehicle to work, an initial set of hyperparameters was selected
based on the prior tests. The is followed by applying the twiddle algorithm to fine tune the hyperparameters.

### Initial PID values
The following initial hyperparameter values were selected based on the vehicle being 
able to travel at a variable throttle of 10-30mph.

| P | I | D |
| --- | --- | --- |
| 0.3 | 0 | 4.5 |

The initial P value of 0.3 was chosen for being sufficient to handle sharp corners and not too large for the 
vehicle to veer into an unstable state (large oscillations) at higher velocities.

The initial D value of 4.5 was chosen for providing the vehicle with a smooth trajectory toward the centre whilst not being too 
large such that the trajectory moves away from the x-axis.

Finally, an initial value of I = 0 was chosen as it was observed that the vehicle had no steering bias.

### Twiddle optimization
Next, the PID control is ran with the Twiddle algorithm. The twiddle algorithm: 
1. Calculates the initial squared error over a pre-defined batch size. This will be the initial 'best error'.
2. Cycles through each coefficient in the following order - P, D, I
3. For each coefficient, first increment the coefficient based on a pre-defined twiddle PID modifier. For example,
if the modified is set to 1, the coefficient will initially be increased by 1.
4. Next the algorithm will run the PID control for the pre-define batch size and then compute the squared error 
over the the batch and compare it to the best error. If it is smaller, the coefficient will be used and the best error
will be updated. The coefficient modifier will be incremented by a factor of 1.1 and the next coefficient will be processed (step 3).
5. If the error is greater than the best error, the coefficient will be decreased based on the pre-defined twiddle modifier.
6. Again, the algorithm will run the PID control for the pre-define batch size and then compute the squared error 
over the the batch and compare it to the best error. If it is smaller, the coefficient will be used and the best error
will be updated. The coefficient modifier will be incremented by a factor of 1.1 and the next coefficient will be processed (step 3).
7. If the error is greater than the best error, the coefficient modifier will be reduced by a factor of 0.9 and the next 
coefficient will be processed (step 3).

After running the twiddle algorithm, the final hyperparameters were set to:

| P | I | D |
| --- | --- | --- |
| 0.347387 | 5.9049e-05 | 4.93047 |

This achieved an squared error rate of approximately 0.00215217.

## Throttle PID
In addition to having the steering controlled by PID. A throttle PID was implemented. As a PID control is meant to
regulate a specific value, in this case the throttle value, proportional to a specific input and based on the tests ran 
with the vehicle at high speeds around the corner. It is assumed that at higher velocity, the vehicle tends to
under-steer and results in the steering control trying very hard to correct its trajectory. As such, the throttle
PID control was designed with the input of the steering value to be inversely proportional to the throttle output.

Specifically, the throttle control PID is inversely proportional to the:
1. The steering value which is linked to the 'Proportional' factor
2. The rate of change of the steering value which is linked to the 'Differential' factor

The sum of steering value, which is linked to the 'Integral' factor was ignored as a track with a bias to turns in one 
direction will force the throttle to be constantly reduced. For example a track the has a lot of right turns will
result in the sum of steering value to be very large value over time, skewing the PID control.

Various P and D values were tests and also parse through the twiddle tuning algorithm and the following final 
hyperparameter was used:

| P | I | D |
| --- | --- | --- |
| 0.9 | 0 | 2.87 |

Additionally, the output of the throttle was scaled to a set minimum and maximum throttle. In the main tests, this was 
set to:
- Minimum : 10mph
- Maximum : 30mph

## Simulations
With the steering PID value set to the tuned hyper parameters and the variable throttle set to top out at 30mph (via the throttle PID) in place,
the simulation was ran with the vehicle successfully navigating the track.

A video demonstration of the simulation can be found [here](https://youtu.be/39WiCC1zcp4)

A high speed test was also successfully ran with an aggressive throttle control and using the same steering PID hyperparameters. 
This was achieved by:
1. Setting a lower inverse P value to 0.8 and increasing the D value to 3.87
2. Adding braking to the throttle control by setting the throttle minimum to -0.2.
3. Setting the maximum throttle to 0.9

This above settings applies maximum throttle when the steering values and rate of change of steering value are low and
apply a negative throttle (braking) when the steering value is large, thus reducing the under-steering.


## Additional Information
Detailed test results can be found in the following [spreadsheet](./results/PID-Control-CTE-tests.xlsx)
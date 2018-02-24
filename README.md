# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
Florian Wulff
---

![png](./visualization/PID_screenshot.jpg)

Videos of the resulting driving around the lake in the simulator can be found in the visualization folder.

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Remarks

This project is built using version 0.14 of uwebsockets. Because of some API change, the main.cpp had to be adapted using pointers for the websocket in all calls. Please consider this when compiling the project. Otherwise, please refer to the videos and use the built binaries.

## Reflection

The PID controller is able to drive around the track endlessly using a constant throttle of 0.3, as implemented in the starter code. It smoothly drive around curves and oscillates just a little bit around the center of the lane.

Higher throttle is possible up to approx. 0.5, reaching a speed of up to 50 mph. This shows a really well choosen parameter set.

### Describe the effect each of the P, I, D components had in your implementation

#### P Parameter

The P is the proportional term. It calculates the steer value directly proportional to the current CTE.

Kp * cte_curr

The P results in an overshooting around zero CTE. Because it operates based on a steady state error, the P term can not compensate a bias.

#### I Parameter

The I is the integral term. It calculates the steer value using the sum of all previous CTE.

Ki * cte_total

The I results in an oscillation around zero CTE. The I term is necessary to compensate for a bias.

#### D Parameter

The D is the dirivative term. It calculates the steer value using the difference between the last timestep's CTE and the current CTE.

Kd * cte_diff

The D results in a strong damping when approach zero CTE and reduces overshooting.

### Describe how the final hyperparameter were chosen

The final hyperparameter were choosen using Twiddle and the given baseline in the lesson of P 0.2, I 0.004 and D 3.0. The values were choosen by Sebastian and can already drive around the track with around 30 mph.

Using twiddle, I calculated the total sum of CTE for approx. one lap and adjusted one of the three parameters by approx. 10%. If the CTE decrease, I keep the parameter change, otherwise the parameter would be decreased. This was repeated until reaching the final values:

* P = 0.24
* I = 0.0036
* D = 3.63

A better way the set the parameter would be to have a physical (motion) model of the car and mathematically calculated the optimal parameter given the car behavior. (mass-spring-damper system)

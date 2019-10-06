# PID Controller

## _A project of Self-Driving Car Engineer Nanodegree_

[![Udacity - Self-Driving Car Engineer Nanodegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)  

### The goals of this project are the following:

- Apply PID control theory to guide a vehicle around a track.
- Understand and tune PID hyper-parameters so that the vehicle can follow the target trajectory smoothly.
- Implement a twiddle (coordinate ascent) algorithm to fine tune the hyper-parameters with reduced cross-track error (CTE).

---

### PID Theory

[PID](https://en.wikipedia.org/wiki/PID_controller) is a negative feedback control. It uses the three control terms of proportional, integral and derivative influence on the controller output to apply accurate and optimal control.

As this project is mainly focused on the steering angle control of a self-driving car, the effects of the PID components explained in below are within the context.

- P, the proportional component
  - steer in proportion to the CTE in reference to the target trajectory
  - larger CTE deserves harder steering
  - can make car overshoot as when the CTE is 0, the steer angle is 0, but the orientation of the car is not 0
  - thus, the car will overshoot
  - this can lead the car into a state called marginal stable, which oscillate around the target trajectory
- I, the integral component
  - compensate systematic bias
  - drive the CTE towards 0
- D, the derivative component
  - causes the steer to decrease if the CTE is increasing rapidly
  - a way to avoid overshoot
  - this will eliminate the oscillation eventually
  - thus, the car is following the target trajectory

### Hyper-parameter Tuning

A combination of manual tuning and twiddler turning is used to look for performing PID coefficients.

- Manual
  - set P, I, D components all to 0s
  - gradually increase P, until the car start to oscillate
  - then, increase D term, until the car stop oscillate and can drive smoothly
  - apply a very small I components to compensate system bias

Here is a great illustration of how the individual components affect the overall system response:

<img src="https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif" alt="PID parameters" width="800">

### Twiddler

Twiddle or coordinate ascent is to find good control gains. It is trying to optimize for a set of parameters. The aim is to minimize the average CTE.

Other than twiddle, there are also other algorithms can be used to fine tune the PID hyper-parameters, such as Ziegler-Nichols Method.

---

### Build the Application

Make sure the script `install-ubuntu.sh` has been executed to install necessary libs that enable communication between this application and the simulator.

Clone this repo and run the following commands:

- `mkdir build && cd build`
- `cmake .. && make`
- `./pid`

### Demo Run

There are two modes to run the application, training mode and racing mode.

- Training Mode
  - This mode is mainly used to tune the PID controller, both manually and using twiddler.

Here is a video recording to show how the PID controller perform in low speed situation.

[![Watch the video](https://img.youtube.com/vi/34LuQF-DiNw/mqdefault.jpg)](https://youtu.be/34LuQF-DiNw)  

- Racing Mode
  - This mode is to apply the fine tuned hyper-parameters for high speed driving.
  - Twiddle turning is disabled.

Here is a video recording to show how the PID controller perform in high speed situation.

[![Watch the video](https://img.youtube.com/vi/l9n7nrzD8bo/mqdefault.jpg)](https://youtu.be/l9n7nrzD8bo)  

#### Extra Safety Features

- A progressive steering compensation has been applied when speed is higher then `40 mph`, and further compensated when speed exceeding `50 mph`.
- A separate PID controller is applied to the throttle control value in regards to a target throttle.
  - Reduce throttle when CTE get larger.
  - Increase throttle when CTE is smaller.

### Reflection

Twiddle algorithm is for a stable system to further minimize the CTE. So, if the system condition is changed, a new set of parameters should be tuned. In this case, when applying the parameters tuned in training mode to high speed situation, we can observe the oscillation, which is expected.

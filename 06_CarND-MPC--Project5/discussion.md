# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program

####1. Student describes their model in detail. This includes the state, actuators and update equations.

In my model, the state includes 6 variables, which are the vehicle's position x and y and rotation psi in vehicles's corordinates. Since I convert the waypoints to vehicle's coordinate's system, these three states are always 0. The 4th state is vehicle's velocity. Another two states are the cross track error(cte) and psi error(epsi). The acuators are vehicle's acceleration and steering angle. The update functions are:

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi_[t+1] = psi[t] -v[t] / Lf * delta[t] * dt

v_[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt

###2. Student discusses the reasoning behind the chosen *N* (timestep length) and *dt* (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The value I chose for N is 10 and dt is 0.1. I use this value because after going through the forum, these combination are suggested to have a relatively better result.  Other values I tried is N = 25 and dt = 0.05, which is from the Udacity quiz.

### 3. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I preprocess the waypoints by first subtracting the vehicle's position and then transformed them into vehicle's coordinate system. In this case, the vehicle is always at (0, 0) in its local coordinate thus the waypoints are transformed to be relative to the origin. And the polynomial fit result will be cleaner.

### 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.


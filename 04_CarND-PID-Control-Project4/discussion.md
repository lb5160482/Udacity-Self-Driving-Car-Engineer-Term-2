# CarND-Controls-PID Discussion
Self-Driving Car Engineer Nanodegree Program

---

## Describes the effect of the P, I, D component of the PID algorithm in their implementation.

* Propotional control is used to set how much we want to directly adjust based on the difference between actual value and desired value. It is propotional because if the error is bigger, than the value we adjust is bigger.
 * Integral control is used to remove the systematic bias. In this case, if the zero angle of the wheel is not actually zero, when we output steering angle to be zero, there will always be an offset. To remove this bias, we use the integral control to take into account all the CTE before to add a control component.
 * Derivitive control is used to remove the bounding effect from propotional control. By simply impolementing a propotional control, the system is hard to converge. In other words, it might bound around the desired value. If we take into account the difference between the two consecutive timestamps and use the difference between the two CTEs, we will be counter the propotional control as  when the propotional control gets a overshoot, we will use derivastive control to decrease the output control from that so that the system might be able to converge later. 

## How did I chose the final hyperparameters (P, I, D coefficients)?

I adjusted the parameters by manual tuning:

- Kp is used to decide how much we want to directly turn based on the error. I tuned the parameter by checking if the vehicle is able to turn enough while it is off the center. If it is overshooting, decrease Kp. If the turning is not enough to counter the offset, increase Kp. I finally set it as 0.2.
- Ki is used to remove the systematic bias. If vehicle is always off the center of the lane, increase Ki. In this project, it seems the vehicle model is good and we do not need an integral control so i set it to be 0.001.
- Kd is used to remove the bounding effect caused by propotional control. After I get a resonable Kp, I tuned Kd by if the vehicle is bounding around lane center, I increase Kd until the vehicle can roughly move along the forward direction. Finally I tuned the value to be 3.0.
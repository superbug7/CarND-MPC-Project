# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## MPC 

Kinematic model is used in this project to approximate the actual vehicle dynamics. It is simpler than dynamic models as it ignores tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable.

The state, actuators and how the state changes over time based on the previous state and current actuator inputs are defined below: 

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
v[t+1] = v[t] + a[t] * dt;
psi[t+1] = psi[t] + v[t] * delta[t] * dt/Lf;

Lf value of 2.67 is chosen. Latency is kept at 100 ms.


### N & dt

Timestamp length (N)  is chosen to be 9 for this project. Time duration between each actuation ( dt ) is chosen to be 0.1. This was the tricky part to adjust for this project. I initially set dt at 0.05 but I could see large steering angle actuations with this value and I could not keep car on track. I took an approach of gradually increasing the duration to see if i get finer control. 0.1 value of dt seems to work best, although I had to adjust N as well, as there is a co-relation. The prediction horizon is almost 1 second, which I thought may be too may be a good range to actuate a vehicle. Although, once other latency oriented factors are brought into control, this may change. 

### Fitting polynomials

As the reference trajectory is passed to the control block as a 3rd order polynomial to fit trajectories for most roads, I fit it to waypoints using Eigen. Basically, I Use polyfit to fit  polynomial to the given x and y coordinates.


### Latency

There is a delay in exeuting an actuation command as there are delays due to command propagation and we need to have that factor included in our model. It mostly is in order of 100 milliseconds, that is why I set it 0.1 seconds as can be seen in equations below. 
This is the advantage of using MPC over PID control. 


          v += throttle * latency_dt;
          px += v * cos(psi) * latency_dt;
          py += v * sin(psi) * latency_dt;
          psi -= v * steering / Lf * latency_dt;
          cte +=  v * sin(epsi) * latency_dt;
          epsi += v * steering / Lf * latency_dt;

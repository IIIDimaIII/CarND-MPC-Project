## Model description

The MPC is designed as a cost optimization problem, where actuators (in the case of current project, steering and throttle) are independent variables and the cost is a function of vehicle current /predicted states deviation from the desired states.

In the project, the vehicle movement model was determined as:

*x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(psi<sub>t</sub>) * dt*

*y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(psi<sub>t</sub>) * dt*

*psi<sub>t+1</sub> = psi<sub>t</sub> + v<sub>t</sub> / L<sub>t</sub> * delta<sub>t</sub> * dt*
 
*v<sub>t</sub> = v<sub>t</sub> + a<sub>t</sub> * dt*
 
*cte<sub>t+1</sub> = f(x<sub>t</sub>) – y<sub>t</sub> + (v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt)*
 
*epsi<sub>t+1</sub> = psi<sub>t</sub> – psi<sub>des t</sub> + (v<sub>t</sub> / L<sub>t</sub> * delta<sub>t</sub> * dt)*
 
*f(x<sub>t</sub>) = at<sub>3</sub>x<sup>3</sup> + at<sub>2</sub>x<sup>2</sup> + at<sub>1</sub>x + at<sub>0</sub>*, where a<sub>3</sub>, a<sub>2</sub>, a<sub>1</sub>, a<sub>0</sub> are determined from fitting  3 order polynomial to waypoints available at time t * 
I considered using 2nd and 3rd order polynomials. During the practical tests 3rd order polynomial showed better results.
 
## Choice of N and dt parameters

After testing various values, I stopped at N = 13 and dt = 0.1. Increasing N while holding dt constant increase predicted horizon and produced better (smooth and precise) planned trajectory (green line) however the vehicle struggled to execute. My guess is that the major problem is imprecise acceleration / deceleration data which creates more problem with longer planning horizon.
When decreasing N, the planned path seems to be “oversimplified”

Lowering dt: higher resolution of the path, but harder to execute.

Combination of N and dt determined the horizon as well as resolution impacting required computing resources and accuracy of the planned path.

## Pre-processing:

**Waypoints** – I converted waypoints to vehicle coordinates. This simplified further calculations, especially tracking vehicle direction with regards to fitted polynomial. (if the waypoints are not converted, we would need to determine if we are going in negative or positive x direction to order to get correct desired psi.)

**State** – velocity from telemetry data should be converted from mph to m/s

**Actuators** - I tested the relation between throttle and acceleration and used linear approximation:

*acceleration = (-0.1132 * v<sub>0</sub> + 5.3603) * throttle;*
## Latency

I addressed the latency issue by shifting state0 – I recalculated x, y, psi, v, cte, epsi assuming the vehicle continues movement for the latency time with same values of actuators as before. Then MPC optimizes path from the recalculated state0.

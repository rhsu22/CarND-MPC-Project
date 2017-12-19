# Notes on the model
This MPC has the following characteristics.

## Reference path and time horizon
The reference path against which control will be predicted is based on a model of the nearby waypoints.  We model this path with a polynomial equation.  With the model coefficients, we can
interpolate and extrapolate the path at points needed for our non-linear optimization.  In our model,
these points are determined by `dt` (the duration between points) and `N` (the number of points).

## Dynamic system model
While the goal is to travel on a path near the reference path, the objectivity of nearness will be
controlled by a cost function.  This cost function is largely based on the position of the vehicle
relative to the path.  In order to determine the position, we need to track the state of the vehicle
at each step.  The state information is summarized by the state vector `[x, y, ψ, v, ε<sub>cte</sub>, ε<sub>ψ</sub>]` (x-position, y-position, vehicle direction, velocity, cross track error, and direction error).  The
state of the vehicle is controlled by two actuator in our model: `[δ, a]` (change in direction and
acceleration).  State-update equations are listed at the end of this section.

## Cost and regulators
The cost being optimized in this system is the sum of the error terms squared.  Additionally, to
ensure that the ride is smooth, the speed between consecutive estimate points should be minimized,
therefore the difference in velocity is also a cost.

To make sure that the actuators are not large (i.e. no flooring the gas paddle) and that there are
no sudden changes in the actuator settings (i.e. no left-right swerving), the actuator values and
the difference in consecutive actuator value are added into the cost as regulators.

The total cost is then:

C = ε<sub>cte</sub><sup>2</sup> + ε<sub>ψ</sub><sup>2</sup> + (v<sub>t</sub> - v<sub>t-1</sub>)<sup>2</sup> + δ<sup>2</sup> + a<sup>2</sup> + (δ<sub>t</sub> - δ<sub>t-1</sub>)<sup>2</sup> + (a<sub>t</sub> - a<sub>t-1</sub>)<sup>2</sup>

## State-update equation and addition notes
x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(ψ<sub>t</sub>) * dt
y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(ψ<sub>t</sub>) * dt
ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> * dt
v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt
ε<sub>cte,t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(ε<sub>ψ,t</sub>) * dt
ε<sub>ψ,t+1</sub> = ψ<sub>t</sub> - *d* ψ(t) + v<sub>t</sub> * δ<sub>t</sub> / L<sub>f</sub> * dt

# Notes on tuning

## Degree of road curvature model
I found that in order to properly model the curvatures reference path, a 3rd degree polynomial is
required.

## N and dt
Along with properly capturing the curvature of the reference path, I also needed to increase the N
to increase the accuracy of the prediction (N = 50, keeping dt at 0.05).

## Weight of steering differential cost
To prevent abrupt changes in steering, I added significant weight to the term
`CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)` (300).

## Accounting for 100 ms latency
It is evident that latency plays a big role in the vehicle's ability to follow the reference path.
This can be easily observed by allowing the vehicle to travel at high speeds and observing that it
has trouble following the path in the default setting (artificial delay) because the actuator values
would be considered too late.  Follow this up by removing the artificial delay and you'll see that
the vehicle travels perfectly well.

One way to account for the delay is to compute for actuator values as if the vehicle is in that
later position.  To do so, offset the vehicle position (`px`, `py`) by adding the distance traveled
in 100 ms.  For convenience, I added the variable `actuator_delay`.

## Actuator limits
The steering wheel is allowed the full range [-1,1].  The acceleration limit has been tuned to be
in the range of [-0.3, 0.3].  I find that with higher values, the amount of speed that the vehicles
will eventually reach is too fast to handle sharp turns.  This may be due to the imprecisions in
predicting for the future (accounting for the artificial sleep).

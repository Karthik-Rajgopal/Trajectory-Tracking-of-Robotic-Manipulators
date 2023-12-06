# Trajectory Tracking of Robotic Manipulators

## Terminology Used:

$q(t), \dot{q}(t), \ddot{q}(t)$ : Position ($rad$), velocity ($rad/ms$) and acceleration ($rad/ms^{2}$) of robot's joints\
$q_{d}(t), \dot{q_{d}}(t), \ddot{q_{d}}(t)$ : Desired position ($rad$), velocity ($rad/ms$) and acceleration ($rad/ms^{2}$) of robot's joints\
$\tau(t)$ : Vector of joint control torques \
$M(q)$ : Inertia matrix of robot\
$C(q, \dot{q})$ : Centrifugal and Coriolis forces coefficient Matrix \
$g(q)$ : Gravity Matrix

## 

Equations of motion of a robot manipulator as described by the Euler-Lagrange model:
$$M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + F_{v} \dot{q} + g(q) = \tau$$

### Proportional-Derivative (PD) Control

The proportional-derivative (PD) feedback control is a fairly famous method in industrial context for the convergence of an output signal to a reference one. It is based on the use of two terms: P and D. The first one affects the system using proportional control on the error introducing a positive matrix $K_{p}$ which is multiplied by the current joints error, i.e. $e(t) = q_{d}(t) - q(t)$. The second term is, instead, the best estimate of the future trend of the error, taking into account its current rate of change, i.e its approximate derivative $\dot{e}(t) = \frac{de}{dt}$, and multiplying it by another positive matrix $K_{d}$. In this case, we are weighting the rate of error change and so how strong the applied control is, based on the rapidity of change. At the end we obtain a control law of the type:
$$u(t)=K_{p}e(t)+K_{d}\dot{e}(t)$$

The full control scheme with the Proportional-Derivative control is shown in Fig.. The PD controller produces a command control that is given as input to the manipulator nominal model shown in Eq. 1. Inverting the Euler-Lagrange equation and substituting $\tau = u$, we get $\ddot{q}$ as:
$$\ddot{q} = M^{-1}(u - C(q, \dot{q}) \dot{q} - F_{v} \dot{q} - g(q))$$
This value is indeed computed in the nominal <em>Robot model scheme</em> block, which takes as input the control command $u$ and that outputs joints acceleration $\ddot{q}$. Velocity ($\dot{q}$) and position ($q$) values of the joints are then obtained using a double integrator as highlighted in the scheme.
![PD Control architecture](https://github.com/Karthik-Rajgopal/Trajectory-Tracking-of-Robotic-Manipulators/blob/main/PID.png)

From these computed quantities, the PD controller makes use of $q_{d}$ and $q$, whose difference produces the current error $e(t)$. Taking the derivative of the error $\dot{e}(t) = \frac{de}{dt}$, it's possible to compute the control law. In general, the advantage of using a PD controller (also in a nonlinear system) is that it is able to reduce as much as possible the error dynamics of the system,
compensating for a possible external disturbance or model mismatch. This is due to the fact that PD control guarantees global stability and so the convergence of an output signal to a desired one. Moreover, this controller is an example of independent and model-free control. This is easy verifiable looking at the control law and noting that it doesn't depend on robot's model.



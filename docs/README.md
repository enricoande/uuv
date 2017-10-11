<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

Welcome to the wiki of the `uuv` project!

Here you can find up-to-date information on the `uuv` repository.

First of all, the repository contains software that is used to simulate and control a single ROV in the Matlab/Simulink environment.

# Current status

The code is not fully functional yet. At the moment, the following items have been implemented:
* Directory in recommended Matlab/Simulink project format;
* ROV 6DOF dynamics block in Simulink (C S-function), including added mass, damping and hydrostatic forces;
* An external current (Matlab/Simulink);
* A thrust block in Simulink (C S-function) that takes propeller revolutions as input and returns the 6DOF thrust vector;
* Plotting and animation functions (Matlab);
* Preprocessing functions (Matlab), including data from the Minerva ROV used at NTNU.

# To-do list

Before the software can be considered as fully functional, the following items are required:
* Path planning methods in Matlab;
* Guidance methods (target & trajectory tracking and path following) in Simulink/Matlab;
* ROV control routines (dynamic positioning and path controller) in Simulink;

# Supported UUVs

## ROV Minerva

The Minerva ROV is in use at NTNU and well documented in the literature (especially in M.Sc. and Ph.D. theses from NTNU).

More information on the ROV can be found here https://www.ntnu.edu/oceans/minerva.

The dynamics coefficients and fits are taken from Mo (2015).

## References

S. M. Mo (2015). _Development of a Simulation Platform for ROV Systems_. Norwegian University of Science and Technology (NTNU), M.Sc. thesis.

# Theory

## UUV dynamics

The notation and equations introduced in Fossen (2011) have been used to program the UUV dynamics. However, the downward right-hand rule convention (north,west,down) is employed at the moment.

Let us define the motions in 6 degrees of freedom (DOF) in the *inertial* reference frame as

$$\mathbf{\eta} = \begin{bmatrix} x & y & z & \phi & \theta & \psi \end{bmatrix}^T ,$$

and the 6DOF velocity vector in the *body-fixed* reference frame as

$$ \mathbf{\nu} = \begin{bmatrix} u & v & w & p & q & r \end{bmatrix}^T .$$

Additionally, it is possible to include the effects of an external current represented by the velocity vector $$\mathbf{\nu}_\mathrm{c}$$, whose 4<sup>th</sup> to 6<sup>th</sup> entries are zero for an irrotational current. The relative velocity vector in body-fixed coordinates is represented by

 $$\mathbf{\nu}_\mathrm{r} = \mathbf{\nu} - \mathbf{\nu}_\mathrm{c}  . $$

Then, the dynamics of an UUV can be expressed by the following system of ordinary differential equations:

$$\begin{bmatrix} \mathbf{\dot{\eta}} \\ \mathbf{\dot{\nu}} \end{bmatrix} = \begin{bmatrix} \mathbf{J}(\mathbf{\eta}) \mathbf{\nu}_\mathrm{r} \\ \left( \mathbf{M}_\mathrm{RB} + \mathbf{M}_\mathrm{A} \right)^{-1} \left( - \mathbf{f}_\mathrm{h} - \mathbf{f}_\mathrm{d} - \mathbf{f}_\mathrm{c} + \mathbf{f}_\mathrm{e} + \mathbf{\tau} \right) \end{bmatrix} ,$$

where $$\mathbf{J}$$ is the transformation matrix for the generalized coordinates, $$\mathbf{M}_\mathrm{RB}$$ the inertia matrix of the rigid body, $$\mathbf{M}_\mathrm{A}$$ the added mass inertia matrix, $$\mathbf{f}_\mathrm{h}$$ the hydrostating restoring force vector, $$\mathbf{f}_\mathrm{d}$$ the damping force vector, $$\mathbf{f}_\mathrm{c}$$ the Coriolis force vector, $$\mathbf{f}_\mathrm{e}$$ the environmental force vector (which includes the tether effects if one is present, e.g. on a ROV) and $$\mathbf{\tau}$$ the thrust force vector.

The restoring force vector is given by

$$ \mathbf{f}_\mathrm{h} = \begin{bmatrix} (W-B) \sin \theta \\ (B-W) \cos \theta \sin \phi \\ (B-W) \cos \theta \cos \phi \\ (y_b B - y_g W\cos \theta \cos \phi + (z_g W -z_b B ) \cos \theta \sin \phi \\ (z_g W - z_b B ) \sin \theta + (x_g W - x_b B ) \cos \theta \cos \phi \\ (x_b B -x_g W ) \cos \theta \sin \phi - (y_b B - y_g W ) \sin \theta \end{bmatrix} , $$

where $$W$$ is the weight force, $$B$$ the buoyancy force, $$\mathrm{COG} = [x_g,y_g,z_g]$$ and $$\mathrm{COB} = [x_b,y_b,z_b]$$.

The damping force vector is represented by a linear and a quadratic term:

$$ \mathbf{f}_\mathrm{d} = \mathbf{D}_\mathrm{l} \mathbf{\nu}_\mathrm{r} + \mathbf{D}_\mathrm{q} \mathrm{diag}\left( | \mathbf{\nu}_\mathrm{r} |\right) \mathbf{\nu}_\mathrm{r} , $$

where $$\mathbf{D}_\mathrm{l}$$ and $$\mathbf{D}_\mathrm{q}$$ are the linear and quadratic damping matrices, respectively.

The Coriolis and centripetal force vector is computed as follows:

$$ \mathbf{f}_\mathrm{c} = \mathbf{C}_\mathrm{RB}(\mathbf{\nu})\mathbf{\nu} + \mathbf{C}_\mathrm{A}(\mathbf{\nu}_\mathrm{r})\mathbf{\nu}_\mathrm{r}  $$

where the first term represents a fictitious force that causes a movement of the UUV relative to the rotating reference frame due to the Earth's rotation and the second term describes the effects related to the added mass. Note that at the moment, this term is commented out.

### ROVs

The $$(6\times 1)$$ thrust force vector is obtained as

$$ \mathbf{\tau} = \mathbf{T} \mathbf{f}_\mathrm{th}, $$

where $$\mathbf{T}$$ is the thrust allocation matrix and $$\mathbf{f}_\mathrm{th}$$ is the thrust force vector that has an entry for every thruster. Hence, the thrust allocation matrix expresses the contribution of each motor in each degree of freedom in the body-fixed coordinate system. Therefore, the thrust allocation matrix is constant if the motors do not rotate.

The thrust force vector is calculated as

$$ \mathbf{f}_\mathrm{th} = \rho \mathbf{D}^4 \odot \mathrm{K}_\mathrm{T} \odot | \mathbf{n} | \odot \mathbf{n} \odot \mathbf{\theta} $$

where all vectors have the length equal to the number of propulsors and $$\odot$$ indicates element-wise multiplication. $$\mathbf{D}$$ is the vector of diameters, $$\mathbf{\theta}$$ is the thrust loss coefficient vector, $$\mathbf{n}$$ is the propulsors' rotational velocity, which is output by the controller, and $$\mathbf{K}_\mathrm{T}$$ is the thrust coefficient vector, which is a function of the advance ratio vector

$$ \mathbf{J}_\mathrm{a} = \frac{\mathbf{\nu}_\mathrm{a}}{\mathbf{n}\mathbf{D}}, $$

where $$\mathbf{\nu}_\mathrm{a}$$ is the vector of the velocity of the propellers through the water.

## LOS guidance


## PID controller

At the moment, very simple PID controllers have been implemented for the control of the surge velocity, $$u$$, the UUV depth, $$z$$, and the heading, $$\psi$$.

The thrust force vector is computed from the inverse of the thrust allocation matrix as follows:

$$ \mathbf{f}_\mathrm{th} = \mathbf{T}^{-1} \mathbf{p}_\mathrm{c} , $$

where $$\mathbf{p}_\mathrm{c}$$ is the $$(6\times 1)$$ vector of control parameters. For the simple PID controller implemented here, the vector of control parameters is given by

$$ \mathbf{p}_\mathrm{c} = \begin{bmatrix} p_\mathrm{c,speed} & 0 & p_\mathrm{c,depth} & 0 & 0 & p_\mathrm{c,heading}  \end{bmatrix}^T. $$

Using simple PID controllers, it is possible to obtain the control parameters for speed, depth and heading as

$$ p_\mathrm{c,speed} = -k_\mathrm{p,speed} \left( u(t)-u_\mathrm{d} (t) \right) - k_\mathrm{i,speed} \int_0^T \left( u(\tau)-u_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,speed} \left( \dot{u}(t)-\dot{u}_\mathrm{d} (t) \right), $$

$$ p_\mathrm{c,depth} = -k_\mathrm{p,depth} \left( z(t)-z_\mathrm{d} (t) \right) - k_\mathrm{i,depth} \int_0^T \left( z(\tau)-z_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,depth} \left( \dot{z}(t)-\dot{z}_\mathrm{d} (t) \right), $$

$$ p_\mathrm{c,heading} = -k_\mathrm{p,heading} \left( \psi(t)-\psi_\mathrm{d} (t) \right) - k_\mathrm{i,heading} \int_0^T \left( \psi(\tau)-\psi_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,heading} \left( \dot{\psi}(t)-\dot{\psi}_\mathrm{d} (t) \right), $$

where $$k_\mathrm{p,speed}$$, $$k_\mathrm{i,speed}$$, $$k_\mathrm{d,speed}$$, $$k_\mathrm{p,depth}$$, $$k_\mathrm{i,depth}$$, $$k_\mathrm{d,depth}$$, $$k_\mathrm{p,heading}$$, $$k_\mathrm{i,heading}$$ and $$k_\mathrm{d,heading}$$ are the proportional, integral and derivative gains for the speed, depth and heading, respectively. Additionally, $$u_\mathrm{d}$$, $$z_\mathrm{d}$$ and $$\psi_\mathrm{d}$$ are the desired speed, depth and heading. At the moment, a very simple scheme is used that relies on a fixed forward speed setting, $U$. In order to prevent a big overshoot in tight corners, the desired speed is set as

$$ u_\mathrm{d} = \lbrace \begin{aligned} U - \frac{U}{\pi/2} |\psi - \psi_\mathrm{d}| \\ 0 \end{aligned}.$$

The desired heading $$\psi_\mathrm{d}$$ and depth $$z_\mathrm{d}$$ are set by the guidance system (at the moment, a simple decoupled line of sight guidance system).

## Path tracking

Path generation and tracking routines are still missing, although a simple minimum-snap trajectory generation and following scheme is implemented for ROVs.

## References
F. Dukan (2014). _ROV Motion Control Systems_. Norwegian University of Science and Technology (NTNU), Ph.D. thesis.

T. I. Fossen (2011). _Handbook of Marine Craft Hydrodynamics and Motion Control_. John Wiley & Sons, first edition.

A. M. Lekkas (2014). _Guidance and Path-Planning Systems for Autonomous Vehicles_. Norwegian University of Science and Technology (NTNU), Ph.D. thesis.

S. M. Mo (2015). _Development of a Simulation Platform for ROV Systems_. Norwegian University of Science and Technology (NTNU), M.Sc. thesis.

## To-do list

* The convention used may be subject to change later on, although care must be taken not to get confused.
* At the moment, transformation matrix relies on the roll, pitch and yaw angles. However, quaternions will be used in the future to prevent instabilities.
* At the moment, the analysed UUVs are assumed not to travel far (in towing tanks or lakes). Hence, the Coriolis force correction has not been implemented yet. This will need to be done.
* At the moment, the tether force for ROVs is not included within the model.

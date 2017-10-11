<script type="text/javascript" async
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js?config=TeX-MML-AM_CHTML">
</script>

Welcome to the wiki of the `uuv` project!

Here you can find up-to-date information on the `uuv` repository.

First of all, the repository contains software that is used to simulate and control a single unmanned underwater vehicle (UUV) in the Matlab/Simulink environment.

# Current status

The code is not fully functional yet. At the moment, the following items have been implemented:
* Directory in recommended Matlab/Simulink project format;
* UUV 6DOF dynamics block in Simulink (C S-function), including added mass, damping, hydrostatic and centripetal (deactivated for the time being) forces and external current effects;
* A thrust block in Simulink (C S-function) that takes propeller revolutions as input and returns the 6DOF thrust vector;
* A simple PID controller in surge, heave and yaw;
* A very basic line of sight guidance system;
* Very basic trajectory generation and following functions;
* Plotting and animation functions (Matlab);
* Preprocessing functions (Matlab), including data from the Minerva ROV used at NTNU.

# To-do list

Before the software can be considered as fully functional, the following items are required:
* Path planning methods in Matlab;
* Guidance methods (target & trajectory tracking and path following) in Simulink/Matlab for both ROVs and AUVs;
* ROV and AUV control routines (dynamic positioning and path controller) in Simulink;
* Thrust models for AUVs.

# Code structure

The code follows a standard Matlab/Simulink project convention. 
The main files and directories are as follows:
* `startup.m`: file that modifies the Matlab path so that all files and directories of interest can be accessed;
* `cleanup.m`: file that clears unwanted clutter and cleans the Matlab path;
* `.\data`: directory that contains the required parameters for the desired UUVs. Note that the data should be saved in `.mat` format as a structure with the same format as the example `rov.mat` file.
* `.\functions`: directory that contains some useful functions;
** `animateAUV.m`: post-processing, low-cost function used to animate the UUV in 3D space in Matlab;
** `plotForces.m`: post-processing function used to plot the forces of interest in Matlab;
** `plotMotions.m`: post-processing function use to plot the motions of interest in Matlab;
** `plotPath.m`: post-processing function to plot the path of the UUV in Matlab;
** `rotation.m`: function to rotate a 3D body in space in an inertial reference frame;
** `skew.m`: function to create a skew-symmetric matrix.
* `.\models`: directory that contains the Simulink models of the UUVs;
** `rov_thrust.c`: C function of the thrust dynamics of a ROV;
** `rovSim_los.slx`: Simulink model of a ROV with line of sight guidance and PID control;
** `uuv_dynamics.c`: C function of the 6DOF dynamics of the UUV;
** `uuvSim_simple`: Simulink model of a simple UUV without thrust block.
* `.\scripts`: directory that contains the main scripts for running the simulations;
** `rovSimRun.m`: function that runs the simulation for the ROV with line of sight guidance and PID control;
** `rovSimSetup.m`: set-up function for the ROV;
** `uuvSimRun.m`: function to run a simple UUV model;
** `uuvSimRun_los.m`: function to run a UUV model with line of sight guidance
** `uuvSimSetup.m`: set-up function for an AUV.

Additionally, the following files and directories may also be of use:
* `.\docs`: directory that contains all documents, such as this one;
* `.\extra`: directory that contains extra files;
** `rovSim_trj.slx`: Simulink model of a ROV with minimum-snap trajectory generation and path following;
** `test_trajectory.m`: file to test the generation of minimum-snap trajectories in 3D space;
** `Trajectory.m`: class used to generate 3D trajectories (incomplete);
** `uuvSimRun_trj.m`: file to run the simulation of a ROV that follows minimum-snap trajectories in 3D space (incomplete).
* `.\preprocessing`: directory that contains pre-processing functions;
** `readData.m`: function used to read input data from UUV parameters and generate a structure compatible with the models used in this project.
* `.\work`: directory that contains all work and temporary files, which will be cleared at the end.

# Quick user guide

This

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

At the moment, only a basic line-of-sight (LOS) guidance system is implemented. A number $$n$$ of waypoints through which the UUV should pass is specified as well as a fixed, non-zero forward speed, $$U$$. Motions in the horizontal and vertical planes are treated as decoupled. The desired heading is then computed in the horizontal plane with a simple line of sight scheme, while in the vertical plane the depth of the points is set as reference.

### Horizontal plane guidance

The desired heading angle is computed as 

$$ \psi_\mathrm{d} = \chi_\mathrm{d} - \beta , $$

where $$\beta=\arcsin \frac{v}{U}$$ is the side slip angle, which compensates for side currents and motions in bends, and $$\chi_\mathrm{d}$$ is the desired course angle. The latter is computed as

$$ \chi_\mathrm{d} = \alpha_k +\chi_\mathrm{r} , $$

where $$ \alpha_k = \arctan \left( \frac{y_{k+1}-y_k}{x_{k+1}-x_k} \right) $$, with $$k$$ indicating the last waypoint that has been encountered and $$k+1$$ the next one, and $$ \chi_\mathrm{r} = \arctan \left( -k_\mathrm{p,steering} e(t) - \int_0^t k_\mathrm{i,steering} e(\tau) \mathrm{d} \tau \right), $$ where the cross-track error $$e(t)$$ is given by

$$ \begin{bmatrix} s(t) \\ e(t) \end{bmatrix} = \begin{bmatrix} \cos \alpha_k & \sin \alpha_k \\ -\sin \alpha_k & \cos \alpha_k \end{bmatrix} \begin{bmatrix} x(t)-x_k \\ y(t)-y_k \end{bmatrix}. $$

### Vertical plane guidance

At the moment, the desired depth is simply set based on the depth of the waypoints as

$$ z_\mathrm{d} = z_{k+1} . $$

However, not that this is valid mainly for ROVs, with AUVs requiring a setting for the pitch angle as well.

## PID controller

At the moment, very simple PID controllers have been implemented for the control of the surge velocity, $$u$$, the UUV depth, $$z$$, and the heading, $$\psi$$.

The thrust force vector is computed from the inverse of the thrust allocation matrix as follows:

$$ \mathbf{f}_\mathrm{th} = \mathbf{T}^{-1} \mathbf{p}_\mathrm{c} , $$

where $$\mathbf{p}_\mathrm{c}$$ is the $$(6\times 1)$$ vector of control parameters. For the simple PID controller implemented here, the vector of control parameters is given by

$$ \mathbf{p}_\mathrm{c} = \begin{bmatrix} p_\mathrm{c,speed} & 0 & p_\mathrm{c,depth} & 0 & 0 & p_\mathrm{c,heading}  \end{bmatrix}^T. $$

Using simple PID controllers, it is possible to obtain the control parameters for speed, depth and heading as

$$ p_\mathrm{c,speed} = -k_\mathrm{p,speed} \left( u(t)-u_\mathrm{d} (t) \right) - k_\mathrm{i,speed} \int_0^t \left( u(\tau)-u_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,speed} \left( \dot{u}(t)-\dot{u}_\mathrm{d} (t) \right), $$

$$ p_\mathrm{c,depth} = -k_\mathrm{p,depth} \left( z(t)-z_\mathrm{d} (t) \right) - k_\mathrm{i,depth} \int_0^t \left( z(\tau)-z_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,depth} \left( \dot{z}(t)-\dot{z}_\mathrm{d} (t) \right), $$

$$ p_\mathrm{c,heading} = -k_\mathrm{p,heading} \left( \psi(t)-\psi_\mathrm{d} (t) \right) - k_\mathrm{i,heading} \int_0^t \left( \psi(\tau)-\psi_\mathrm{d} (\tau) \right) \mathrm{d} \tau - k_\mathrm{d,heading} \left( \dot{\psi}(t)-\dot{\psi}_\mathrm{d} (t) \right), $$

where $$k_\mathrm{p,speed}$$, $$k_\mathrm{i,speed}$$, $$k_\mathrm{d,speed}$$, $$k_\mathrm{p,depth}$$, $$k_\mathrm{i,depth}$$, $$k_\mathrm{d,depth}$$, $$k_\mathrm{p,heading}$$, $$k_\mathrm{i,heading}$$ and $$k_\mathrm{d,heading}$$ are the proportional, integral and derivative gains for the speed, depth and heading, respectively. Additionally, $$u_\mathrm{d}$$, $$z_\mathrm{d}$$ and $$\psi_\mathrm{d}$$ are the desired speed, depth and heading. At the moment, a very simple scheme is used that relies on a fixed forward speed setting, $U$. In order to prevent a big overshoot in tight corners, the desired speed is set as

$$ u_\mathrm{d} = \begin{cases} U - \frac{U}{\pi/2} |\psi - \psi_\mathrm{d}| \quad \text{if } |\psi-\psi_\mathrm{d}| \leq \frac{\pi}{2} ,  \\ 0  \quad \text{if } |\psi-\psi_\mathrm{d}| > \frac{\pi}{2} . \end{cases} $$

The desired heading $$\psi_\mathrm{d}$$ and depth $$z_\mathrm{d}$$ are set by the guidance system (at the moment, a simple decoupled line of sight guidance system).

## Path tracking

Path generation and tracking routines are still missing, although a simple minimum-snap trajectory generation and following scheme is implemented for ROVs.

## References
F. Dukan (2014). _ROV Motion Control Systems_. Norwegian University of Science and Technology (NTNU), Ph.D. thesis.

T. I. Fossen (2011). _Handbook of Marine Craft Hydrodynamics and Motion Control_. John Wiley & Sons, first edition.

A. M. Lekkas (2014). _Guidance and Path-Planning Systems for Autonomous Vehicles_. Norwegian University of Science and Technology (NTNU), Ph.D. thesis.

S. M. Mo (2015). _Development of a Simulation Platform for ROV Systems_. Norwegian University of Science and Technology (NTNU), M.Sc. thesis.

## To-do list

* Path planning and tracking tools will need to be created.
* The convention used may be subject to change later on, although care must be taken not to get confused. Note that the animation displays the UUV in a NEU inertial frame.
* At the moment, transformation matrix relies on the roll, pitch and yaw angles. However, quaternions can be used in the future to prevent instabilities for angles close to $$90^\circ$$.
* At the moment, the analysed UUVs are assumed not to travel far (in towing tanks or lakes). Hence, the Coriolis force correction has not been implemented yet. This will need to be done.
* At the moment, the tether force for ROVs is not included within the model.

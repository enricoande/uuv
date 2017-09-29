{% include mathjax.html %}

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

# Theory

## UUV dynamics

The notation and equations introduced in Fossen (2011) have been used to program the UUV dynamics. However, the upward right-hand rule convention (north,west,up) is used instead of (north,east,down) at the moment.

Let us define the motions in 6 degrees of freedom (DOF) in the *inertial* reference frame as

$$ \mathbf{\eta} = \begin{bmatrix} x & y & z & \phi & \theta & \psi \end{bmatrix}^T ,$$

and the 6DOF velocity vector in the *body-fixed* reference frame as

$$ \mathbf{\nu} = \begin{bmatrix} u & v & w & p & q & r \end{bmatrix}^T .$$

Then, the dynamics of an UUV can be expressed by the following system of ordinary differential equations:

$$ \begin{bamtrix} \mathbf{\dot{\eta}} // \mathbf{\dot{\nu}} \end{bmatrix} = \begin{bamtrix} \mathbf{J}(\mathbf{\eta}) \mathbf{\nu} // \left( \mathbf{M}_\mathrm{RB} + \mathbf{M}_\mathrm{A} \right)^{-1} \mathbf{f}_\mathrm{h} + \mathbf{f}_\mathrm{d} + \mathbf{f}_\mathrm{C} + \mathbf{f}_\mathrm{e} + \mathbf{\tau} \end{bmatrix} ,$$

where:

### References
T. I. Fossen (2011). _Handbook of Marine Craft Hydrodynamics and Motion Control_. John Wiley & Sons, first edition.

### To-do list

* convention
* quaternions
* Coriolis force

# 6DOF Spacecraft Simulator

A C++ spacecraft dynamics simulator that models both translational and rotational motion using vector mathematics, force modeling, torque-based angular acceleration, and quaternion attitude integration.

This project is currently being expanded to include a Raylib frontend for visualization and future AI/LLM API capabilities for interactive simulation guidance, mission analysis, and user assistance.

## Project Overview

This simulator models spacecraft motion in six degrees of freedom:

* 3 translational degrees of freedom: x, y, z position
* 3 rotational degrees of freedom: roll, pitch, yaw attitude motion

The program accepts user-defined spacecraft parameters, computes forces and accelerations, updates position over time, and integrates spacecraft orientation using quaternions.

## Core Features

* 3D vector operations
* Matrix-vector multiplication
* Net force calculation
* Translational motion integration
* Angular acceleration calculation
* Quaternion-based attitude integration
* User-defined spacecraft mass, velocity, torque, inertia, thrust, drag coefficient, and timestep
* Position and quaternion history output over simulation steps

## Physics and Simulation Components

### Translational Dynamics

The simulator computes spacecraft acceleration using Newton’s Second Law:

```text
a = F_net / m
```

The net force model includes:

* Thrust force
* Gravity force
* Drag force

Position is updated iteratively using velocity, acceleration, timestep, and simulation step count.

### Rotational Dynamics

The simulator computes angular acceleration using:

```text
α = I⁻¹τ
```

where:

* `I` is the spacecraft inertia matrix
* `τ` is the applied torque vector
* `α` is angular acceleration

### Quaternion Attitude Integration

The simulator uses quaternions to represent spacecraft orientation and avoid limitations of Euler angles such as gimbal lock.

The orientation is updated using angular velocity and quaternion multiplication, then normalized each timestep to maintain numerical stability.

## Current Architecture

The project is organized around:

* `sim.cpp`: Handles user input, spacecraft state setup, simulation execution, and output.
* `math.cpp`: Contains custom vector, matrix, force, motion, and quaternion math functions.
* `Vec3.h` / `sim.h`: Header files for shared structures and function declarations.

## Example Inputs

The simulator accepts:

* Spacecraft mass
* Initial position vector
* Initial quaternion orientation
* Initial velocity vector
* Initial angular velocity vector
* Initial acceleration vector
* Timestep size
* Number of simulation steps
* Thrust value
* Torque vector
* Diagonal inertia matrix values
* Unit thrust direction vector
* Spacecraft area
* Drag coefficient

## Example Outputs

The simulator prints:

* Quaternion values over time
* Computed acceleration
* Position values over time

## Technologies Used

* C++
* Custom Vector and Matrix Math
* Quaternion Rotation Mathematics
* Classical Mechanics
* Planned: Raylib
* Planned: LLM API Integration

## Author

Veltman Okey-Ejiowhor

Mechanical Engineering Student | AI/ML Developer | Aerospace and Simulation Enthusiast

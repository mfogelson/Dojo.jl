# Trajectory Optimization

!!! info 
    The trajectory optimization examples are outdated and currently not available.

Dojo provides dynamics constraints and Jacobians in order to perform trajectory optimization using [iterative LQR](https://github.com/thowell/IterativeLQR.jl). 

## Quadruped

```@raw html
<img src="../assets/animations/quadruped_min.gif" width="200"/>
```

A [Unitree A1](https://www.unitree.com/products/a1/) takes a number of forward steps. There are costs on a kinematic gait and control usage, as well as an augmented Lagrangian (i.e., soft) constraint on the the robot's final pose. The maximal representation is converted to a minimal one for optimization. Additionally, slack controls are utilized early on to aid the optimizer before being driven to zero by a constraint to achieve a dynamically feasible trajectory.

## Atlas

```@raw html
<img src="../assets/animations/atlas_ilqr.gif" width="200"/>
```

The Atlas v5 humanoid (sans arms) takes a number of forward steps. Similar to the quadruped example, there are costs on control effort and deviations from a kinematic plan, a minimal representation is utilized, and the optimizer is aided by slack controls.

## Block

```@raw html
<img src="../assets/animations/box_right.gif" width="200"/>
```

A block is moved to a goal location by applying forces to its center of mass. The optimizer is initialized with zero control and utilizes smooth gradients from Dojo to find a motion that overcomes friction to slide towards the goal.

## Raibert Hopper

```@raw html
<img src="../assets/animations/hopper_max.gif" width="100"/>
```

A hopping robot, inspired by the [Raibert Hopper](https://dspace.mit.edu/handle/1721.1/6820), is tasked with moving to a goal location. The optimizer finds a single hop trajectory to reach its goal pose.

## Cartpole

```@raw html
<img src="../assets/animations/cartpole_max.gif" width="200"/>
```

This classic system is tasked with performing a swing-up. Examples are provided performing optimization with both maximal and minimal representations. 
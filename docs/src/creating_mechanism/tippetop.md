# Detailed Mechanism Definition
Here, we describe in detail how to define your own dynamical system of [`Mechanism`](@ref). After it has been defined, it will be extremely easy to simulate it, control it, perform trajectory optimization on it, or even policy optimization.

We're going to build a tippe top:


```@raw html
<img src="../assets/animations/tippetop_real.gif" width="300"/>
```

```@raw html
<img src="../assets/animations/tippetop.gif" width="300"/>
```

### Build Mechanism

We will take a look at the definition of `get_tippetop` in `DojoEnvironments`. This function return a [`Mechanism`](@ref) and takes as input a variety of parameters like the simulation time step, gravity etc. You can add as many parameters you want. This example is typical of what you will find in Dojo.

To build the mechanism corresponding to the tippe top, we decompose it into two spherical bodies. Each body has its own spherical contact constraint with the floor. The joint between the two bodies is a `Fixed` joint and the joint between the main body and the [`Origin`](@ref) of the frame is a `Floating` joint.

```julia
function get_tippetop(;
    timestep=0.01,
    input_scaling=timestep, 
    gravity=-9.81,
    mass=1,
    radius=0.5,
    scale=0.2,
    color=RGBA(0.9, 0.9, 0.9, 1.0),
    springs=0,
    dampers=0, 
    limits=false,
    joint_limits=Dict(),
    keep_fixed_joints=false, 
    friction_coefficient=0.4,
    contact=true,
    contact_type=:nonlinear,
    T=Float64)

    # mechanism
    origin = Origin{T}(name=:origin)
    
    bodies = [
        Sphere(radius, mass; name=:sphere1, color),
        Sphere(radius*scale, mass*scale^3; name=:sphere2, color)
    ]
    bodies[1].inertia = Diagonal([1.9, 2.1, 2])

    joints = [
        JointConstraint(Floating(origin, bodies[1]); name=:floating_joint),
        JointConstraint(Fixed(bodies[1], bodies[2];
            parent_vertex=[0,0,radius]), name = :fixed_joint)
    ]

    mechanism = Mechanism(origin, bodies, joints;
        timestep, gravity, input_scaling)

    # springs and dampers
    set_springs!(mechanism.joints, springs)
    set_dampers!(mechanism.joints, dampers)

    # joint limits    
    if limits
        joints = set_limits(mechanism, joint_limits)

        mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints;
            gravity, timestep, input_scaling)
    end
    
    # contacts
    contacts = ContactConstraint{T}[]

    if contact
        n = length(bodies)
        normals = fill(Z_AXIS,n)
        friction_coefficients = fill(friction_coefficient,n)
        contact_radii = [radius;radius*scale]
        contacts = [contacts;contact_constraint(bodies, normals; friction_coefficients, contact_radii, contact_type)]
    end

    mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, contacts;
        gravity, timestep, input_scaling)

    # zero configuration
    initialize_tippetop!(mechanism)

    # construction finished
    return mechanism
end
```

### Initialize Mechanism
The second method that we need to look at is `initialize_tippetop!`. This function initialize the dynamical system to a certain state. This means that we set the position orientation, linear and angular velocity of each body in the mechanism.


```julia
function initialize_tippetop!(mechanism::Mechanism{T};
    body_position=2*Z_AXIS*mechanism.bodies[1].shape.r, body_orientation=one(Quaternion),
    body_linear_velocity=zeros(3), body_angular_velocity=[0.0, 0.1, 50.0]) where T

    # zero state
    zero_velocities!(mechanism)
    zero_coordinates!(mechanism)

    # set desired state value 
    floating_joint = mechanism.joints[1]
    set_minimal_coordinates!(mechanism, floating_joint, 
        [body_position; Dojo.rotation_vector(body_orientation)])
    set_minimal_velocities!(mechanism, floating_joint, 
        [body_linear_velocity; body_angular_velocity])
end
```

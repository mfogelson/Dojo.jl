function get_cartpole(; 
    timestep=0.01, 
    input_scaling=timestep, 
    gravity=-9.81, 
    slider_mass=1,
    pendulum_mass=1,
    link_length=1,
    radius=0.075,
    color=RGBA(0.7, 0.7, 0.7, 1),
    springs=0, 
    dampers=0,
    joint_limits=Dict(),
    keep_fixed_joints=true, 
    T=Float64)

    # mechanism
    origin = Origin{Float64}()
    slider = Capsule(1.5 * radius, 1, slider_mass; 
        orientation_offset=Dojo.RotX(0.5 * π), color, name=:cart)
    pendulum = Capsule(radius, link_length, pendulum_mass; color, name=:pole)
    bodies = [slider, pendulum]
    
    joint_origin_slider = JointConstraint(Prismatic(origin, slider, Y_AXIS); name=:cart_joint)
    joint_slider_pendulum = JointConstraint(Revolute(slider, pendulum, X_AXIS; 
        child_vertex=-0.5*link_length*Z_AXIS); name=:pole_joint)
    joints = [joint_origin_slider, joint_slider_pendulum]

    mechanism = Mechanism(origin, bodies, joints;
        gravity, timestep, input_scaling)

    # springs and dampers
    set_springs!(mechanism.joints, springs)
    set_dampers!(mechanism.joints, dampers)

    # joint limits    
    joints = set_limits(mechanism, joint_limits)
    mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints;
        gravity, timestep, input_scaling)

    # zero configuration
    initialize_cartpole!(mechanism)

    # construction finished
    return mechanism
end

function initialize_cartpole!(mechanism::Mechanism; 
    position=0, orientation=pi/4)

    zero_velocities!(mechanism)
    zero_coordinates!(mechanism)
    
    set_minimal_coordinates!(mechanism, mechanism.joints[1], [position])
    set_minimal_coordinates!(mechanism, mechanism.joints[2], [orientation])

    return
end
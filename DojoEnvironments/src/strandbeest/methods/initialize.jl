function get_strandbeest(;
    timestep=0.01,
    gravity=[0.0; 0.0; -9.81],
    friction_coefficient=1.0,
    contact_foot=true,
    contact_body=true,
    limits=true,
    model=:strandbeest,
    floating=true,
    contact_type=:nonlinear,
    spring=0.0,
    damper=0.0,
    parse_damper=true,
    T=Float64)

    path = joinpath(@__DIR__, "../deps/$(String(model)).urdf")
    mech = Mechanism(path; floating, T,
        gravity,
        timestep,
        parse_damper)

    # Adding springs and dampers
    set_springs!(mech.joints, spring)
    set_dampers!(mech.joints, damper)

    # joint limits
    joints = deepcopy(mech.joints)

    if limits
        joint1 = get_joint(mech, :joint1)
        joint3 = get_joint(mech, :joint3)
        joints[joint1.id] = add_limits(mech, joint1,
            rot_limits=[SVector{1}(-0.7597), SVector{1}(1.8295)])
        joints[joint3.id] = add_limits(mech, joint3,
            rot_limits=[SVector{1}(-1.8295), SVector{1}(0.7597)])
        mech = Mechanism(Origin{T}(), [mech.bodies...], [joints...];
            gravity,
            timestep)
    end

    if contact_foot
        origin = Origin{T}()
        bodies = mech.bodies
        joints = mech.joints

        normal = [0.0; 0.0; 1.0]
        models = []

        link3 = get_body(mech, :link3)
        link2 = get_body(mech, :link2)
        foot_radius = 0.0203
        ankle_radius = 0.025
        base_radius = 0.14
        p = [0.1685; 0.0025; -0.0055]
        o = foot_radius
        push!(models, contact_constraint(link3, normal;
            friction_coefficient,
            contact_origin=p,
            contact_radius=o,
            contact_type,
            name=:foot))
        p = [-0.10; -0.002; 0.01]
        o = ankle_radius
        push!(models, contact_constraint(link3, normal;
            friction_coefficient,
            contact_origin=p,
            contact_radius=o,
            contact_type,
            name=:ankle3))
        p = [0.24; 0.007; 0.005]
        push!(models, contact_constraint(link2, normal;
            friction_coefficient,
            contact_origin=p,
            contact_radius=o,
            contact_type,
            name=:ankle2))
        base_link = get_body(mech, :base_link)
        pl = [0.0; +0.075; 0.03]
        pr = [0.0; -0.075; 0.03]
        o = base_radius
        push!(models, contact_constraint(base_link, normal;
            friction_coefficient,
            contact_origin=pl,
            contact_radius=o,
            contact_type,
            name=:torso_left))
        push!(models, contact_constraint(base_link, normal;
            friction_coefficient,
            contact_origin=pr,
            contact_radius=o,
            contact_type,
            name=:torso_right))

        set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0,0,1.0, 0,0,0])

        mech = Mechanism(origin, bodies, joints, [models...];
            gravity,
            timestep)
    end
    return mech
end

function initialize_rexhopper!(mechanism::Mechanism{T};
        body_position=zeros(3),
        body_orientation=zeros(3),
        body_linear_velocity=zeros(3),
        body_angular_velocity=zeros(3)) where T

    zero_velocity!(mechanism)
    body_position += [0.0, 0.0, 0.3148]

    for joint in mechanism.joints
        !(joint.name in (:loop_joint, :floating_joint)) && set_minimal_coordinates!(mechanism, joint, zeros(input_dimension(joint)))
    end

    set_minimal_coordinates!(mechanism, get_joint(mechanism,
        :floating_base), [body_position; body_orientation])
    set_minimal_velocities!(mechanism, get_joint(mechanism,
        :floating_base), [body_linear_velocity; body_angular_velocity])
end

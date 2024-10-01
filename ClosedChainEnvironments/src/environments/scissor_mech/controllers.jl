using Dojo

"""
Calculate the angle between two bodies in the mechanism.

Parameters:
- body1: First body
- body2: Second body

Returns:
- roll, pitch, yaw: Angles of rotation about the x, y, and z axes respectively
"""
function get_angle_between_bodies(body1, body2)
    state1 = body1.state
    state2 = body2.state

    q1 = state1.q2
    q2 = state2.q2

    R1 = Dojo.rotation_matrix(q1)
    R2 = Dojo.rotation_matrix(q2)

    R_rel = R2 * R1'

    roll = atan(R_rel[3,2], R_rel[2,2])
    pitch = atan(-R_rel[1,3], sqrt(R_rel[2,3]^2 + R_rel[3,3]^2))
    yaw = atan(R_rel[1,2], R_rel[1,1])

    return roll, pitch, yaw
end

"""
Controller that tries to maintain a target angle between the first two bodies.

Parameters:
- mechanism: The Dojo Mechanism object
- k: Current timestep (unused in this controller)
"""
function angle_controller!(mechanism, k)
    target_angle = pi/2
    current_angle, _, _ = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])
    
    angle_error = target_angle - current_angle
    link_length = mechanism.bodies[1].shape.rh[2]

    vel_error = -mechanism.bodies[1].state.ω15[1]

    Kp = 1.0
    Kd = 0.7

    control_torque = Kp * -angle_error + Kd * vel_error
    control_force = control_torque / link_length / 2

    add_external_force!(mechanism.bodies[1], force=[0, control_force/2, 0], vertex=[0, 0, -link_length/2])
    add_external_force!(mechanism.bodies[2], force=[0, -control_force/2, 0], vertex=[0, 0, -link_length/2])
end

"""
Controller that applies spring forces to maintain a target length between bodies.

Parameters:
- mechanism: The Dojo Mechanism object
- k: Current timestep (unused in this controller)
"""
function spring_controller!(mechanism, k)
    l0 = 0.010 # resting length of the spring (m)
    num_cells = 2
    link_length = mechanism.bodies[1].shape.rh[2]  # This should be parameterized
    k = 2870  # spring constant (N/m) 1lb / cm = 2.54 lb / in = 28.7 N / cm = 2870 N / m

    for i in 1:num_cells
        body1 = mechanism.bodies[2*i-1]
        body2 = mechanism.bodies[2*i]

        p1 = body1.state.x2 + Dojo.rotation_matrix(body1.state.q2) * [0, 0, link_length/2]
        p2 = body2.state.x2 + Dojo.rotation_matrix(body2.state.q2) * [0, 0, link_length/2]
        l = Dojo.norm(p1 - p2)

        control_force = k * (l - l0)
        if l < l0
            control_force = 0
        end

        control_torque = control_force * link_length / 2

        # println(control_force)

        # add_external_force!(body1, force=Dojo.vector_rotate([0, -control_force/2, 0], body1.state.q2'), vertex=[0, 0, link_length/2])
        # add_external_force!(body2, force=Dojo.vector_rotate([0, control_force/2, 0], body2.state.q2'), vertex=[0, 0, link_length/2])
        add_external_force!(body1, torque=[(-1)^(i+1)*control_torque/2, 0, 0])
        add_external_force!(body2, torque=[(-1)^(i)*control_torque/2, 0, 0])
    end
end
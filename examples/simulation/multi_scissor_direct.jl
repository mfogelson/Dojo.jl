# ### Setup
# PKG_SETUP
using Pkg; Pkg.activate(".")
using Dojo
using JLD2
using Dates
# ### Mechanism components
function get_scissor_mechanism(num_sets=3, initial_angle=0.0, slop=0.0)
    # ### Parameters
    radius = 0.03
    link_length = 0.45 # m? 
    mass = 0.0015 # kg? 
    rotation_axis = [1;0;0] 
    damper = 0.001

    origin = Origin()

    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    loop_joints = JointConstraint{Float64}[]
    rot_joint_limits_even = [[-pi], [0]]
    rot_joint_limits_odd = [[0], [pi]]
    for i in 1:num_sets
        body = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i-1)"))
        body2 = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i)"))

        push!(bodies, body)
        push!(bodies, body2)

        if i == 1
            joint1 = JointConstraint(Revolute(origin, body, rotation_axis), name=Symbol("origin_joint"))
            joint2 = JointConstraint(Revolute(body, body2, rotation_axis, rot_joint_limits=rot_joint_limits_odd), name=Symbol("joint_pairs$i"))
            push!(joints, joint1)
            push!(joints, joint2)
        else
            if iszero(slop)
                joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
                joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
            else
                tra_joint_limits = [[-slop/2, -slop/2], [slop/2, slop/2]]
                joint1 = JointConstraint(PlanarAxis(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], tra_joint_limits=tra_joint_limits, damper=damper))
                joint2 = JointConstraint(PlanarAxis(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], tra_joint_limits=tra_joint_limits, damper=damper))
            end
            push!(joints, joint1)
            push!(joints, joint2)

            if iszero(slop)
                joint3 = JointConstraint(Revolute(body, body2, rotation_axis, rot_joint_limits=iseven(i) ?  rot_joint_limits_even : rot_joint_limits_odd, damper=damper), name=Symbol("joint_pairs$i"))
            else
                tra_joint_limits = [[-slop/2, -slop/2], [slop/2, slop/2]]
                joint3 = JointConstraint(PlanarAxis(body, body2, rotation_axis; tra_joint_limits=tra_joint_limits, rot_joint_limits=iseven(i) ?  rot_joint_limits_even : rot_joint_limits_odd, damper=damper), name=Symbol("joint_pairs$i"))
            end

            push!(loop_joints, joint3)
        end
    end


    # ### Construct Mechanism
    append!(joints, loop_joints)
    mechanism = Mechanism(origin, bodies, joints, timestep=0.001, gravity=[-9.81, 0.0, 0.0])
    # set_dampers!(mechanism.joints, 0.001)

    exlude_ids = [j.id for j in loop_joints]
    # ### Set state
    # for i in 1:num_sets
    #     set_minimal_coordinates!(mechanism, joints[2*i-1], [(-1)^(i+1)*pi/4], exclude_ids=exlude_ids)
    #     set_minimal_coordinates!(mechanism, joints[2*i], [(-1)^(i)*pi/4], exclude_ids=exlude_ids)
    # end

    for i in 1:num_sets
        Dojo.set_maximal_configurations!(mechanism.bodies[2i-1], x=[0, 0, (i-1)*link_length*cos(initial_angle/2)], q=Dojo.RotX((-1)^(i+1)*initial_angle/2))
        Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=[0, 0, (i-1)*link_length*cos(initial_angle/2)], q=Dojo.RotX((-1)^(i)*initial_angle/2))
    end
    return mechanism
end

# set_minimal_coordinates!(mechanism, joints[1], [pi/4], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[2], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[3], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[4], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[5], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[6], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[7], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[8], [pi/2], exclude_ids=exlude_ids)


# set_minimal_velocities!(mechanism, joints[1], [0.5])
# set_minimal_velocities!(mechanism, joints[2], [-0.5])
# set_maximal_velocities!(mechanism.bodies[1], ω=[50.0;0;0])
# set_maximal_velocities!(mechanism.bodies[2], ω=[-50.0;0;0])

function get_angle_between_bodies(body1, body2)
    # Get the current states of the two bodies
    state1 = body1.state
    state2 = body2.state

    # Extract the orientations (assuming they're represented as quaternions)
    q1 = state1.q2
    q2 = state2.q2

    # Convert quaternions to rotation matrices
    R1 = Dojo.rotation_matrix(q1)
    R2 = Dojo.rotation_matrix(q2)

    # Calculate the relative rotation between the two bodies
    R_rel = R2 * R1'

    # Extract the angle of rotation about the x-axis
    roll = atan(R_rel[3,2], R_rel[2,2])
    pitch = atan(-R_rel[1,3], sqrt(R_rel[2,3]^2 + R_rel[3,3]^2))
    yaw = atan(R_rel[1,2], R_rel[1,1])

    return roll, pitch, yaw
end


function controller!(mechanism, k)
    # Target angle in radians
    target_angle = pi/2

    # Get the current angle between the two bodies
    current_angle, _, _ = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])

    # println("Current angle: ", current_angle)

    # Calculate the error
    angle_error = target_angle - current_angle
    link_length = 0.45

    vel_error = -mechanism.bodies[1].state.ω15[1]
    # println("Angle error: ", angle_error)

    # Proportional gain (you may need to tune this)
    Kp = 1.0
    Kd = 0.7

    # Calculate the control torque
    control_torque = Kp * -angle_error + Kd * vel_error
    control_force = control_torque/link_length/2

    # println("Control torque: ", control_torque)
    # println("Control force: ", control_force)

    # Apply the control torque to both bodies in opposite directions around x-axis
    # add_external_force!(mechanism.bodies[1], force=[0.0, 0.0, 0.0], torque=[control_torque/2, 0.0, 0.0])
    # add_external_force!(mechanism.bodies[2], force=[0.0, 0.0, 0.0], torque=[-control_torque/2, 0.0, 0.0])

    # You can keep the original forces if needed, or modify them
    add_external_force!(mechanism.bodies[1], force=[0, control_force/2, 0], vertex=[0, 0, -link_length/2])
    add_external_force!(mechanism.bodies[2], force=[0, -control_force/2, 0], vertex=[0, 0, -link_length/2])
end

# # set_minimal_velocities!(mechanism, joint12, [0.2])

# mechanism.gravity = [0;0;9.8]

# ### Simulate
# Regularization
# for i in 1:mechanism.system.matrix_entries.n
#     mechanism.system.matrix_entries[i,i].value += Dojo.I*1e-6
# end

function main()
    println("Initialized Mechanism")
    cells = 20
    slop = 0.003
    theta0 = -pi*9/10
    mechanism = get_scissor_mechanism(cells, theta0, slop)
    
    println("Starting Simulation")
    cur_angle = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])
    steps = 500
    storage = Storage(steps, length(mechanism.bodies))
    start = now()
    simulate!(mechanism, 1:steps, storage, controller!, record=true , opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-10,verbose=false))
    println("Simulation Done")
    println("Runtime: ", now()-start)
    
    datetime = now()
    println("Saving Data")
    save("scissor_cells_$(cells)_slope_$(slop)_theta0_$(round(theta0, digits=2))_$datetime.jld2", "mechanism", mechanism, "storage", storage)
end
main()


# mechanism, storage = load("scissor_cells_3_slope_0.004_theta0_-2.83_2024-09-11T22:39:35.902.jld2", "mechanism", "storage")
# vis
# ### Visualize
# vis = Visualizer()
# delete!(vis)
# visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true)
# vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)
# render(vis)

# for body in mechanism.bodies
#     println(body.name)
#     q = body.state.q2
#     w, x, y, z = q.s, q.v1, q.v2, q.v3

#     # Convert to Euler angles (yaw, pitch, roll) in radians
#     pitch = asin(2.0 * (w * y - z * x))
#     yaw = atan(2.0 * (w * z + x * y), 1.0 - 2.0 * (y^2 + z^2))
#     roll = atan(2.0 * (w * x + y * z), 1.0 - 2.0 * (x^2 + y^2))
#     println("Pitch: ", roll)
# end

# for joint in mechanism.joints
#     println(joint.name)
#     println("joint angle: ", Dojo.minimal_coordinates(mechanism, joint))
# end
# ### Setup
# PKG_SETUP
using Pkg; Pkg.activate(".")
using Dojo
using JLD2
using Dates
using Plots
# ### Mechanism components
function get_scissor_mechanism(num_sets=3, initial_angle=0.0, slop=0.0)
    # ### Parameters
    radius = 0.003
    link_length = 0.045 # m? 
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


function spring_controller!(mechanism, k)
    # resting length of the spring
    l0 = 0.0206375 # m
    num_cells = 2
    link_length = 0.045
    for i in 1:num_cells
        body1 = mechanism.bodies[2*i-1]
        body2 = mechanism.bodies[2*i]

        # current length of the spring
        p1 = body1.state.x2 + Dojo.rotation_matrix(body1.state.q2) * [0, 0, link_length/2]
        p2 = body2.state.x2 + Dojo.rotation_matrix(body2.state.q2) * [0, 0, link_length/2]
        l = Dojo.norm(p1 - p2)

        # spring constant
        k = 1.0 # N/m

        control_force = k * (l - l0)
        println("Control force: ", control_force)
    
        add_external_force!(body1, force=[0, control_force/2, 0], vertex=[0, 0, link_length/2])
        add_external_force!(body2, force=[0, -control_force/2, 0], vertex=[0, 0, link_length/2])
    end
end

# # set_minimal_velocities!(mechanism, joint12, [0.2])

# mechanism.gravity = [0;0;9.8]

# ### Simulate
# Regularization
# for i in 1:mechanism.system.matrix_entries.n
#     mechanism.system.matrix_entries[i,i].value += Dojo.I*1e-6
# end
cells = 23
slop = 0.001
theta0 = -pi*9/10
println("Initialized Mechanism with $cells cells")

mechanism = get_scissor_mechanism(cells, theta0, slop)
solver = Dojo.mehrotra_svd!
println("Starting Simulation with $solver")
mechanism.timestep = 1/240
steps = 61

# cur_angle = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])
storage = Storage(steps, length(mechanism.bodies))
# start = now()
simulate!(mechanism, 1:steps, storage, spring_controller!, record=true , opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-10,verbose=false, svd_threshold=1e-7), abort_upon_failure=true, solver=solver)
println("Simulation Done")
vis = Visualizer()
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)
# plot the position and end points of each of the members for all time
m = steps
n = 3*length(mechanism.bodies)
body_pos_meters = [[0.0, 0.0, 0.0] for i in 1:m, j in 1:n]
for body in 1:length(mechanism.bodies)
    for timestep in 1:steps
        # Extract the position at the current timestep for the current body
        pos = storage.x[body][timestep]
        pos_endpoint = storage.x[body][timestep] + Dojo.rotation_matrix(storage.q[body][timestep]) * [0, 0, -0.045/2]
        pos_endpoint2 = storage.x[body][timestep] + Dojo.rotation_matrix(storage.q[body][timestep]) * [0, 0, 0.045/2]
        # Store the position in the body_pos_meters array
        body_pos_meters[timestep, 3*body-2] = pos
        body_pos_meters[timestep, 3*body-1] = pos_endpoint
        body_pos_meters[timestep, 3*body] = pos_endpoint2
    end
end

using CSV
using DataFrames
CSV.write("paper_data/Scissor_jamming/09_25_2024_scissor_mechanism_slop_0.001_K_1.0.csv", DataFrame(body_pos_meters, :auto))
body_pos_meters = CSV.read("paper_data/Scissor_jamming/scissor_mechanism.csv", DataFrame)


# load csv file 
# Read the CSV file
body_pos_df = CSV.read("/Users/mitchfogelson/Projects/Research_Projects/co-tracker/videos/pred_tracks_formatted.csv", DataFrame)
# remove the first column
body_pos_df = select(body_pos_df, Not(:Frame))

# Function to parse the coordinate string
function parse_coord(s::AbstractString)
    # Remove parentheses and split by comma
    x, y = split(replace(s, r"[()]" => ""), ",")
    return parse(Float64, x), parse(Float64, y)
end

# Get the number of frames and nodes
n_frames = nrow(body_pos_df)
n_nodes = ncol(body_pos_df)   # Subtract 1 for the 'Frame' column

# Initialize the output array
body_pos_meters_real = zeros(Float64, n_frames, n_nodes, 2)

# Parse each coordinate pair and fill the array
for (i, row) in enumerate(eachrow(body_pos_df))
    for j in 1:ncol(body_pos_df)  # Start from 2 to skip the 'Frame' column
        x, y = parse_coord(row[j])
        # idx = (i - 1) * n_nodes + (j - 1)
        body_pos_meters_real[i, j, 1] = x
        body_pos_meters_real[i, j, 2] = y
    end
end


# Function to parse the coordinate string
function parse_coord(s::AbstractString)
    # Remove brackets and split by comma
    return parse.(Float64, split(replace(s, r"[\[\]]" => ""), ","))
end
# Extract coordinates from the first row
coords = [parse_coord(body_pos_meters[1, col]) for col in names(body_pos_meters)]

# Separate x, y, and z coordinates
x = [coord[1] for coord in coords]
y = [coord[2] for coord in coords]
z = [coord[3] for coord in coords]

using Plots
# scatter plot the first row of data 
scatter(body_pos_meters_real[1, :, 1]*55 .- 0.1, body_pos_meters_real[1, :, 2]*55 .- 0.7, label="Initial Position", xlabel="x (m)", ylabel="z (m)", title="Scissor Mechanism Position Over Time", legend=:topleft, aspect_ratio=:equal)
scatter!(z, y, label="Real Data")
# scatter(body_pos_meters[1, :, 2], body_pos_meters[1, :, 3], label="Initial Position", xlabel="x (m)", ylabel="z (m)", title="Scissor Mechanism Position Over Time", legend=:topleft, aspect_ratio=:equal)
# scatter!(body_pos_meters[1, :, :], label="Real Data")
# println("Runtime: ", now()-start)


# Load the predicted data
body_pos_meters = CSV.read("paper_data/Scissor_jamming/09_25_2024_scissor_mechanism_slop_0.001_K_1.0.csv", DataFrame)

# Load the real data
body_pos_df = CSV.read("/Users/mitchfogelson/Projects/Research_Projects/co-tracker/videos/pred_tracks_formatted.csv", DataFrame)
body_pos_df = select(body_pos_df, Not(:Frame))

# Function to parse the coordinate string for real data
function parse_coord_real(s::AbstractString)
    x, y = split(replace(s, r"[()]" => ""), ",")
    return parse(Float64, x), parse(Float64, y)
end

# Function to parse the coordinate string for predicted data
function parse_coord_pred(s::AbstractString)
    return parse.(Float64, split(replace(s, r"[\[\]]" => ""), ","))
end

# Process real data
n_frames = nrow(body_pos_df)
n_nodes = ncol(body_pos_df)
body_pos_meters_real = zeros(Float64, n_frames, n_nodes, 2)

for (i, row) in enumerate(eachrow(body_pos_df))
    for j in 1:ncol(body_pos_df)
        x, y = parse_coord_real(row[j])
        body_pos_meters_real[i, j, 1] = x
        body_pos_meters_real[i, j, 2] = y
    end
end

# Process predicted data
coords_pred = [parse_coord_pred(body_pos_meters[i, col]) for i in 1:nrow(body_pos_meters), col in names(body_pos_meters)]
x_pred = [coord[1] for coord in coords_pred]
y_pred = [coord[2] for coord in coords_pred]
z_pred = [coord[3] for coord in coords_pred]

# Create the animation
anim = @animate for i in 1:n_frames
    scatter(z_pred[i, :], y_pred[i, :], 
            label="Predicted Slop=0.001 K=1 N/m", markersize=7, color=:red)
    scatter!(body_pos_meters_real[i, :, 1]*5.5 .-0.01, body_pos_meters_real[i, :, 2]*5.5 .- 0.07, 
            label="Real", xlabel="x (m)", ylabel="z (m)", 
            title="Scissor Mechanism Position - Frame $i", 
            legend=:topleft, aspect_ratio=:equal,
            markersize=6, color=:blue)
    
    xlims!(-0.03, .3)  # Adjust these limits as needed
    ylims!(-0.06, 0.06)  # Adjust these limits as needed
end

# Save the animation
gif(anim, "paper_data/Scissor_jamming/scissor_mechanism_animation_slop_0.001.gif", fps = 30)

datetime = now()
println("Saving Data")

function main()
    steps = 500
    # cells = 1
    slop = 0.0
    theta0 = -pi*9/10
    for solver in [Dojo.mehrotra!, Dojo.mehrotra_svd!, Dojo.mehrotra_niave!]
        for cells in 1:20
            println("Initialized Mechanism with $cells cells")

            mechanism = get_scissor_mechanism(cells, theta0, slop)

            println("Starting Simulation with $solver")
            # cur_angle = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])
            storage = Storage(steps, length(mechanism.bodies))
            # start = now()
            simulate!(mechanism, 1:steps, storage, controller!, record=true , opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-10,verbose=false, svd_threshold=1e-7), abort_upon_failure=true, solver=solver)
            println("Simulation Done")
            # println("Runtime: ", now()-start)

            datetime = now()
            println("Saving Data")
            # remove Dojo.
            # solver =
            save("/Users/mitchfogelson/.julia/dev/Dojo.jl/paper_data/Scissor_Sweep/$(solver)/scissor_cells_$(cells)_slope_$(slop)_theta0_$(round(theta0, digits=2))_$datetime.jld2", "mechanism", mechanism, "storage", storage)
        end
    end
end
main()


# mechanism, storage = load("linkage_slop_data/scissor_slop/scissor_cells_20_slope_0.003_theta0_-2.83_2024-09-12T08:03:36.199.jld2", "mechanism", "storage");
# mechanism, storage = load("linkage_slop_data/scissor_slop/scissor_cells_3_slope_0.004_theta0_-2.83_2024-09-11T22:39:35.902.jld2", "mechanism", "storage");
mechanism, storage = load("paper_data/Scissor_jamming/09_25_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism.jld2", "mechanism", "storage");

save("paper_data/Scissor_jamming/10_3_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism.jld2", "mechanism", mechanism, "storage", combine_storage)

mechanism, combine_storage = load("paper_data/Scissor_jamming/10_3_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism.jld2", "mechanism", "storage");
vis
delete!(vis)
visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)
function set_maximal_state!(mechanism::Mechanism, storage::Storage; ind=1)
    for (body, x, q, ω, v) in zip(mechanism.bodies, storage.x, storage.q, storage.ω, storage.v)
        body.state.x1 = x[ind]
        body.state.x2 = x[ind]
        body.state.q1 = q[ind]
        body.state.q2 = q[ind]
        body.state.ω15 = ω[ind]
        body.state.v15 = v[ind]
        body.state.d -= body.state.d
        body.state.D -=  body.state.D
        body.state.Fext -= body.state.Fext
        body.state.τext -= body.state.τext
        body.state.vsol[1] -= body.state.vsol[1] 
        body.state.vsol[2] -= body.state.vsol[2]
        body.state.ωsol[1] -= body.state.ωsol[1]
        body.state.ωsol[2] -= body.state.ωsol[2]
    end
end
steps = size(storage.x[1])[1]

set_maximal_state!(mechanism, storage, ind=1)
Dojo.reset!.(mechanism.joints, scale=1.0)
include("/Users/mitchfogelson/.julia/dev/Dojo.jl/ClosedChainEnvironments/src/environments/scissor_mech/controllers.jl")
new_storage = Storage(275, length(mechanism.bodies))
include("/Users/mitchfogelson/.julia/dev/Dojo.jl/ClosedChainEnvironments/src/environments/scissor_mech/scissor_mechanism.jl")

mechanism = get_scissor_mechanism(num_sets=23, damper=0.01, initial_angle=-pi*8/10, timestep=0.001, slop=0.001)
simulate!(mechanism, 1:275, new_storage, spring_controller!, record=true , opts=SolverOptions(rtol=1e-6, btol=1e-4, reg=1e-8,verbose=false, svd_threshold=1e-6), abort_upon_failure=true, solver=Dojo.mehrotra_svd!)
mechanism.timestep
mechanism.gravity
delete!(vis)
vis = visualize(mechanism, combine_storage; vis=vis, visualize_floor=false, show_frame=true)
set_maximal_state!(mechanism, combine_storage, ind=steps+31)
for joint in mechanism.joints
    println(joint.name)
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    println(cbody.name)
    origin_parent = pbody.state.x2+Dojo.vector_rotate( joint.translational.vertices[1], pbody.state.q2)
    origin_child = cbody.state.x2+Dojo.vector_rotate(joint.translational.vertices[2], cbody.state.q2)
    if pbody.name != Symbol("origin")
        parent_forces = Dojo.impulse_map(mechanism, joint, pbody)*joint.impulses[2]
        Dojo.set_arrow!(vis, origin_parent, Dojo.vector_rotate(parent_forces[1:3], pbody.state.q2), scaling=0.1, color=RGBA(1, 0, 0, 0.5), name=pbody.name)
    end
    child_forces = Dojo.impulse_map(mechanism, joint, cbody)*joint.impulses[2]
    println(origin_child)
    Dojo.set_arrow!(vis, origin_child, Dojo.vector_rotate(child_forces[1:3], cbody.state.q2), shaft_radius=0.001, max_head_radius=0.01, scaling=1.0, color=RGBA(0, 1, 0, 0.5), name=joint.name)
end

delete!(vis)


# combine storages 
combine_storage = Storage(steps+39, length(mechanism.bodies))
for i in 1:steps
    for j in 1:length(mechanism.bodies)
        combine_storage.x[j][i] = storage.x[j][i]
        combine_storage.q[j][i] = storage.q[j][i]
        combine_storage.ω[j][i] = storage.ω[j][i]
        combine_storage.v[j][i] = storage.v[j][i]
    end
end
start = steps+1 
end_step = steps+39
for i in start:end_step
    for j in 1:length(mechanism.bodies)
        combine_storage.x[j][i] = new_storage.x[j][i-steps]
        combine_storage.q[j][i] = new_storage.q[j][i-steps]
        combine_storage.ω[j][i] = new_storage.ω[j][i-steps]
        combine_storage.v[j][i] = new_storage.v[j][i-steps]
    end
end
vis = visualize(mechanism, combine_storage; vis=vis, visualize_floor=false, show_frame=true)

SVDs = []
for i in 1:steps
    set_maximal_state!(mechanism, storage, ind=i)
    Dojo.set_entries!(mechanism)
    A = full_matrix(mechanism.system)
    F = Dojo.svd(A, full=true, alg=Dojo.LinearAlgebra.QRIteration())
    push!(SVDs, F)
end

for i in 1:39
    set_maximal_state!(mechanism, new_storage, ind=i)
    Dojo.set_entries!(mechanism)
    A = full_matrix(mechanism.system)
    F = Dojo.svd(A, full=true, alg=Dojo.LinearAlgebra.QRIteration())
    push!(SVDs, F)
end

using Plots
plot([F.S for F in SVDs], yscale=:log10, legend=false, title="Singular Values of Scissor with 3 cells 0.004 Slop", xlabel="Index", ylabel="Singular Value")

plot([sum(F.S .< 1e-6) for F in SVDs], legend=false, label="Total Singular Values", title="Number of Singular Values less than 1e-6", xlabel="Step", ylabel="Number of Singular Values less than 1e-6", lw=3)

A = full_matrix(mechanism.system)
F = Dojo.svd(A, full=true, alg=Dojo.LinearAlgebra.QRIteration())
using Plots
plot(F.S, yscale=:log10, legend=false, title="Singular Values of Scissor with 3 cells 0.004 Slop", xlabel="Index", ylabel="Singular Value")

for i in [1, 244]
    set_maximal_state!(mechanism, storage, ind=i)
    Dojo.set_entries!(mechanism)
    for joint in mechanism.joints Dojo.input_impulse!(joint, mechanism) end
    joint = mechanism.joints[1]
    println(joint.impulses)
end


# vis
# ### Visualize
vis = Visualizer()
delete!(vis)
# visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)
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
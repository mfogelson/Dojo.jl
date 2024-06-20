# Pseudocode for System Identification
using Dojo
using Pkg
Pkg.activate("DojoEnvironments")
using DojoEnvironments
Pkg.activate(".")
using JLD2
using CSV
using DataFrames 

# Step 2: Define System Model
# Define a function, SystemModel, that takes mass, damping, force, and time as inputs and outputs predicted position and orientation.
 ### Parameters
# Link params
radius = 0.0127
link_length = 0.14605
max_extension = 1.1684
mass = 0.001 #TODO

# Plate params
plate_radius = 0.5842/2
plate_thickness = 0.00635
plate_mass = 0.001 #TODO

# Sim params
damper = 0.01
timestep = 0.005
reg = 1e-8
gravity = [0, 0, -9.81]

# System Params
num_cells = 2

mechanism_dict = Dict(:radius=>radius, :link_length=>link_length, :mass=>mass, :plate_radius=>plate_radius, :plate_thickness=>plate_thickness, :plate_mass=>plate_mass, :num_cells=>num_cells, :timestep=>timestep, :gravity=>gravity, :max_extension=>max_extension, :dampers=>damper)

mechanism = create_mechanism(;mechanism_dict...)

# Step 1: Load Data
# Load CSV data related to the position and orientation of the plates
# Julia load csv data 
file_path = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/01_HERDS_Fogelson_Thomas_Falcon_Lipton_Manchester/Kresling_Deploy_Run_01/natnet_ros-HexagonMiddle-pose.csv"

# Read the CSV file into a DataFrame
df = CSV.File(file_path) |> DataFrame
position_middle = df[!, ["pose.position.x", "pose.position.y", "pose.position.z"]]
orientation_middle = df[!, ["pose.orientation.w", "pose.orientation.x", "pose.orientation.y", "pose.orientation.z"]]

file_path = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/01_HERDS_Fogelson_Thomas_Falcon_Lipton_Manchester/Kresling_Deploy_Run_01/natnet_ros-HexagonTop-pose.csv"

# Read the CSV file into a DataFrame
df = CSV.File(file_path) |> DataFrame
position_top = df[!, ["pose.position.x", "pose.position.y", "pose.position.z"]]
orientation_top = df[!, ["pose.orientation.w", "pose.orientation.x", "pose.orientation.y", "pose.orientation.z"]]

# Save the data into storage 
start_ind = 13500
end_ind = 15500
steps = end_ind - start_ind+1
mechanism = create_mechanism(;mechanism_dict...)

# Rotate and translate mechanism 
function move_mechanism!(mechanism, Δx, Δq)
    for body in mechanism.bodies
        x = Dojo.vector_rotate(body.state.x2, Δq) + Δx
        q = Δq * body.state.q2
        set_maximal_configurations!(body, x=x, q=q)
    end
end
set_maximal_configurations!(get_body(mechanism, Symbol("plate_2")), x=[0.0, 0.0, position_middle[1,3]]) #, q=Quaternion(orientation_middle[1,1], orientation_middle[1,2], orientation_middle[1,3], orientation_middle[1,4]))
set_maximal_configurations!(get_body(mechanism, Symbol("plate_3")), x=[0.0, 0.0, position_top[1,3]])#, q=Quaternion(orientation_middle[1,1], orientation_middle[1,2], orientation_middle[1,3], orientation_middle[1,4]))

Δq = Quaternion(orientation_middle[1,1], orientation_middle[1,2], orientation_middle[1,3], orientation_middle[1,4])

move_mechanism!(mechanism, [0.0, 0.0, 0.0], Δq/norm(Δq))
delete!(vis)
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.01)
plate_ids = [get_body(mechanism, Symbol("plate_$(i)")).id for i in 1:num_cells+1]
res = initialize_constraints!(mechanism, 
                                    fixedids=plate_ids,
                                    regularization=0.0,
                                    lineIter=10, 
                                    newtonIter=10,
                                    debug=true,
                                    ε = 1e-6)

storage = Storage(steps, length(mechanism.bodies))

for (i, (x_middle, x_top, q_middle, q_top)) in enumerate(zip(eachrow(position_middle[start_ind:end_ind, :]), eachrow(position_top[start_ind:end_ind, :]), eachrow(orientation_middle[start_ind:end_ind, :]), eachrow(orientation_top[start_ind:end_ind, :])))
    set_maximal_configurations!(get_body(mechanism, Symbol("plate_2")), x=Vector(x_middle)-[Vector(position_middle[1,1:2]);0.0], q= Quaternion(q_middle[1], q_middle[2], q_middle[3], q_middle[4]))
    

    set_maximal_configurations!(get_body(mechanism, Symbol("plate_3")), x=Vector(x_top)-[Vector(position_top[1,1:2]); 0.0], q= Quaternion(q_top[1], q_top[2], q_top[3], q_top[4]))

    res = initialize_constraints!(mechanism, 
                                    fixedids=plate_ids,
                                    regularization=0.0,
                                    lineIter=10, 
                                    newtonIter=10,
                                    debug=false,
                                    ε = 1e-5)

    Dojo.save_to_storage!(mechanism, storage, i)
    # visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)

end

visualize(mechanism, storage; vis=vis, visualize_floor=true, show_frame=false, show_joint=false, joint_radius=0.01)

# set_maximal_configurations!(get_body(mechanism, Symbol("plate_1")), x=zeros(3), q=Quaternion(1.0, 0.0, 0.0, 0.0))


Dojo.max_violations(mechanism)
delete!(vis)


for body in mechanism.bodies
    set_maximal_configurations!(mechanism.origin, body, Δx=[1.0, 0.0, 0.0], Δq=Quaternion(orientation_middle[1,1], orientation_middle[1,2], orientation_middle[1,3], orientation_middle[1,4]))
end


storage = Storage(steps, length(mechanism.bodies))

storage2 = Storage(steps, length(mechanism.bodies))


# index = [1 if body.name == Symbol("middle_plate") else 2 if body.name == Symbol("top_plate") else 0 for body in mechanism.bodies]

for (i, (x_middle, x_top, q_middle, q_top)) in enumerate(zip(eachrow(position_middle[start_ind:end_ind, :]), eachrow(position_top[start_ind:end_ind, :]), eachrow(orientation_middle[start_ind:end_ind, :]), eachrow(orientation_top[start_ind:end_ind, :])))
    storage.x[2][i] = Vector(x_middle) - [position_middle[1, 1]; position_middle[1, 2]; 0.0]
    storage.q[2][i] = Quaternion(q_middle[1], q_middle[2], q_middle[3], q_middle[4])
    storage.x[3][i] = Vector(x_top) - [position_top[1, 1]; position_top[1, 2]; 0.0]
    storage.q[3][i] = Quaternion(q_top[1], q_top[2], q_top[3], q_top[4])

    storage2.x[1][i] = [position_middle[1, 1]; position_middle[1, 2]; 0.0]
    storage2.q[1][i] = Quaternion(1.0, 0.0, 0.0, 0.0)
    storage2.x[2][i] = Vector(x_middle) 
    storage2.q[2][i] = Quaternion(q_middle[1], q_middle[2], q_middle[3], q_middle[4])
    storage2.x[3][i] = Vector(x_top)
    storage2.q[3][i] = Quaternion(q_top[1], q_top[2], q_top[3], q_top[4])

end

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end

vis, animation = visualize(mechanism, storage; return_animation=true, vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05, name=:original)
mechanism = create_mechanism(;mechanism_dict...)

mechanism.timestep = 0.005
Tf = 20*mechanism.timestep
timesteps = 0:mechanism.timestep:Tf
steps = length(timesteps)
storage2 = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=false, reg=reg, max_iter=10)
simulate!(mechanism, 1:steps, storage2, control!, record=true, opts=opts)
delete!(vis)
visualize(mechanism, storage2; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05, name=:learned)

# Step 3: Define Cost Function
# Define a function, CostFunction, that calculates the difference (e.g., mean squared error) between the SystemModel outputs and the actual data from Trial 1.
function get_z_prediction(mechanism)
    middle = get_body(mechanism, Symbol("plate_$(num_cells)"))
    top = get_body(mechanism, Symbol("plate_$(num_cells+1)"))

    return [middle.state.x2; middle.state.q2; top.state.x2; top.state.q2]

end

function get_z_true(position_middle, orientation_middle, position_top, orientation_top, idx)
    return [position_middle[idx, :]; orientation_middle[idx, :]; position_top[idx, :]; orientation_top[idx, :]]
end

function cost(z_prediction, z_true, Q)
    return 0.5 * (z_prediction - z_true)'* Q *(z_prediction - z_true)
end

# Step 4: Parameter Optimization
# Initialize mass, damping, and force with initial guesses.
mass_guess = cat([[0.001; I(3)...] for _ in mechanism.bodies]...) #[kg] / I
damping_guess = ones(length(mechanism.joints))*0.01 #[N/m/s]
force_guess = [zeros(3) for _ in 1:size(df)[1]] #[N]
# Use an optimization algorithm (e.g., gradient descent) to minimize the CostFunction, adjusting mass, damping, and force.
# get gradient of simulation with respect to mass, damping, and force
z0 = get_maximal_state(mechanism)
function step!(mechanism::Mechanism{T}, z::Vector{T}, u::Vector{T}, m::Vector{T}, d::Vector{T}; 
    opts=SolverOptions{T}()) where T
    # set_mass and set_damper 
    for (i, body) in enumerate(mechanism.bodies)
        set_mass!(body, m[10*(i-1)+1:i*10], mechanism.timestep)
    end

    for (i, joint) in enumerate(mechanism.joints)
        set_dampers!([joint], d[i])
    end

    # set state
    set_maximal_state!(mechanism, z)

    # set control
    # set_input!(mechanism, u)
    top = get_body(mechanism, Symbol("plate_3"))
    set_external_force!(top, force=u)

    # solve the 1-step simulation problem
    mehrotra!(mechanism, opts=opts)
    for body in mechanism.bodies 
        Dojo.update_state!(body, mechanism.timestep) 
    end

    # extract the next state
    z_next = Dojo.get_next_state(mechanism)
    
    return z_next
end
z_prediction = step!(mechanism, z0, force_guess[1], mass_guess, damping_guess, opts=opts)

using ForwardDiff
using FiniteDiff
FiniteDiff.finite_difference_jacobian(x -> step!(mechanism, z0, x, mass_guess, damping_guess, opts=opts), force_guess[1])
FiniteDiff.finite_difference_jacobian(x -> step!(mechanism, z0, force_guess[1], x, damping_guess, opts=opts), mass_guess)


# Store the optimized parameters.

# Step 5: Validate Model
# Use the optimized parameters in SystemModel to predict position and orientation for Trials 2 and 3.
# Calculate and compare the difference between these predictions and the actual data for Trials 2 and 3.
# Assess the model's accuracy and generalizability based on these comparisons.


## Track position in the simulation and then extract the forces from the real data 
using Dojo
using Pkg
Pkg.activate("./DojoEnvironments")
using DojoEnvironments
Pkg.activate(".")
using LinearAlgebra
using Optim

# Function to create a snake mechanism
function create_snake(; dampers=0.3, joint_type=:Revolute)
    mech = DojoEnvironments.get_snake(
        gravity=[0.0, 0.0, 0.0], num_bodies=2, 
        dampers=dampers, joint_type=joint_type)
    DojoEnvironments.initialize_snake!(mech)
    return mech
end

# Simulate the snake and return the trajectory
function simulate_snake(mech, duration, num_steps)
    function ctrl!(m, k)
        # if k == 1
        #     Dojo.set_input!(m.joints[2], [100.0])
        # end
        Dojo.set_input!(m.joints[2], [1000.0]) #[sin(k)*1.0])
    end
    
    storage = Dojo.simulate!(mech, duration, ctrl!, record = true, verbose=false)
    return storage
end

# Extract positions from state trajectory
function extract_positions(trajectory)
    return [state[1:end] for state in trajectory]  # Assuming first 3 elements are positions
end

# Compute residuals between simulated and target trajectories
function compute_residuals(damping, target_trajectory, duration, num_steps)
    # mech = create_snake(dampers=damping)
    # simulated_trajectory = simulate_snake(mech, duration, num_steps)
    mech = get_scissor_mechanism(num_sets=10, damper=damping, initial_angle=-pi*8/10, slop=0.001)
    simulated_trajectory = Storage(num_steps, length(mech.bodies))
    simulated_trajectory = simulate!(mech, 1:num_steps, simulated_trajectory, spring_controller!, 
            record=true, 
            opts=SolverOptions(rtol=1e-5, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-6),
            abort_upon_failure=true,
            solver=Dojo.mehrotra_svd!)
    simulated_positions = extract_positions(simulated_trajectory.x)
    target_positions = extract_positions(target_trajectory)
    
    residuals = vcat([sim - target for (sim, target) in zip(simulated_positions, target_positions)]...)
    println("Residuals: ", dot(residuals, residuals))
    return residuals
end

# Objective function for optimization
function objective(damping, target_trajectory, duration, num_steps)
    residuals = compute_residuals(damping[1], target_trajectory, duration, num_steps)
    return dot(residuals, residuals)
end

# Function to estimate damping using LBFGS
# function estimate_damping(target_trajectory, duration, num_steps; initial_damping=0.1)
#     obj = (damping) -> objective(damping, target_trajectory, duration, num_steps)
#     result = optimize(obj, [initial_damping], LBFGS(), Optim.Options(show_trace = true))
#     return Optim.minimizer(result)[1]
# end
function estimate_damping(target_trajectory, duration, num_steps; initial_damping=0.1, initial_angle=-pi*8/10)
    obj = (damping) -> objective(damping, target_trajectory, duration, num_steps)
    
    # Set up custom stopping criteria
    options = Optim.Options(show_trace = true, g_tol = 1e-5, x_tol = 1e-5, f_tol = 1e-5)
    lb = [0.000001]
    ub = [1.0]
    result = optimize(obj, lb, ub, [initial_damping], Fminbox(LBFGS()), options)
    return result, Optim.minimizer(result)[1]
    
end

# Generate synthetic data
true_damping = 1000.0
true_angle
duration = 1.0
num_steps = 100
mech_true = create_snake(dampers=true_damping)
target_trajectory = simulate_snake(mech_true, duration, num_steps)

using Plots
scatter()
for x in target_trajectory.x[1]
    scatter!([x[1]], [x[2]], [x[3]], label="", color="blue")
end
for x in target_trajectory.x[2]
    scatter!([x[1]], [x[2]], [x[3]], label="", color="red")
end
scatter!()
true_damping = 100.0
duration = 1.0
num_steps = 100
mech_true = create_snake(dampers=true_damping)
target_trajectory = simulate_snake(mech_true, duration, num_steps)
for x in target_trajectory.x[1]
    scatter!([x[1]], [x[2]], [x[3]], label="", color="green")
end
for x in target_trajectory.x[2]
    scatter!([x[1]], [x[2]], [x[3]], label="", color="black")
end
scatter!()

# vis = Visualizer()
delete!(vis)
vis = visualize(mech_true, target_trajectory, vis=vis, visualize_floor=false, show_joint=true, joint_radius=0.01, show_frame=true)
# Estimate damping
estimated_damping = estimate_damping(target_trajectory.x, duration, num_steps)

println("True damping: ", true_damping)
println("Estimated damping: ", estimated_damping)

# Compute final loss
final_residuals = compute_residuals(estimated_damping, target_trajectory.x, duration, num_steps)
final_loss = dot(final_residuals, final_residuals)
println("Final loss: ", final_loss)

# Compute the Jacobian for the final estimated mechanism
final_mech = create_snake(dampers=estimated_damping)
final_trajectory = simulate_snake(final_mech, duration, num_steps)

joint = final_mech.joints[2]  # Assuming we're interested in the second joint
pbody = final_mech.bodies[1]
cbody = final_mech.bodies[2]
final_jacobians = Dojo.body_constraint_jacobian_body_data(final_mech, pbody, cbody, joint)
println("Constraint Jacobians:")
println("Parent wrt Parent: ", final_jacobians[1])
println("Parent wrt Child: ", final_jacobians[2])

include("scissor_mechanism.jl")
include("controllers.jl")
true_damping = 0.001
true_angle = -pi*8/10
@time true_scissor = get_scissor_mechanism(num_sets=10, initial_angle=true_angle, damper=true_damping, slop=0.001)

steps = 60
storage = Storage(steps, length(true_scissor.bodies))
    
@time simulate!(true_scissor, 1:steps, storage, spring_controller!, 
            record=true, 
            opts=SolverOptions(rtol=1e-5, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-6),
            abort_upon_failure=true,
            solver=Dojo.mehrotra_svd!)

vis = Visualizer()
vis = visualize(true_scissor, storage, vis=vis, visualize_floor=false, show_joint=true, joint_radius=0.005, show_frame=true)

# Estimate damping
estimated_damping = estimate_damping(storage.x, steps*true_scissor.timestep, steps, initial_damping=0.01)
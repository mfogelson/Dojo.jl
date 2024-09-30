using Distances
using Hungarian
using LinearAlgebra
using ForwardDiff
using Optim
using Dojo

include("scissor_mechanism.jl")
# # Include the distance_func from your previous code here
# # function distance_func(...) end

# function simulate_mechanism(slop, damping, initial_angle, timesteps)
#     # This is a placeholder function. You'll need to implement the actual simulation logic.
#     # It should return simulated_data with shape (4*timesteps, num_nodes, 3)
#     get_scissor_mechanism(;
#     num_sets=3,
#     initial_angle=initial_angle,
#     slop=slop,
#     link_length=0.045,
#     link_mass=0.0015,
#     link_radius=0.003,
#     damper=damping, 
#     rotation_axis=[1; 0; 0], 
#     gravity=[-9.81, 0, 0],
#     timestep=0.001,
#     rot_joint_limits_even=[[-π+0.02], [0.02]],
#     rot_joint_limits_odd=[[0.02], [π-0.02]]
#     )

#     storage = Storage(timesteps, length(mechanism.bodies))
#     simulate!(mechanism, 1:timesteps, storage, spring_controller!, 
#               record=true, 
#               opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-7),
#               abort_upon_failure=true,
#               solver=Dojo.mehrotra!)
#     simulated_data = zeros(eltype(slop, damping), steps, length(mechanism.bodies)*3, 3)

#     for step in 1:steps
#         for body in 1:length(mechanism.bodies)
#             simulated_data[step, 3*body-2, :] = storage.x[body][step] + Dojo.rotation_matrix(storage.q[body][step]) * [0, 0, -0.045/2]
#             simulated_data[step, 3*body-1, :] = storage.x[body][step] + Dojo.rotation_matrix(storage.q[body][step]) * [0, 0, 0.045/2]
#             simulated_data[step, 3*body, :] = storage.x[body][step] 
#         end
#     end
#     return simulated_data
# end

# function cost(params, real_data, initial_state, ordering)
#     slop, damping = params
#     simulated_data = simulate_mechanism(slop, damping, initial_state, size(real_data, 1))
    
#     dists = Float64[]
#     for n in 1:size(real_data, 1)
#         v1 = [real_data[n, i,:]*5.5 - [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
#         v2 = [[simulated_data[4*(n-1)+1, i,3], simulated_data[4*(n-1)+1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]
#         dist = sum(norm(v1[j] - v2[ordering[j]]) for j in 1:size(real_data, 2))
#         push!(dists, dist)
#     end
#     return sum(dists)
# end

# function optimize_parameters(real_data, simulated_data, initial_state)
#     # Compute initial ordering
#     v1 = [real_data[1, i, :] .* 5.5 .- [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
#     v2 = [[simulated_data[1, i,3], simulated_data[1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]
#     ordering, _ = distance_func(hcat(v1...), hcat(v2...))


#     # Define the objective function
#     objective(params) = cost(params, real_data, initial_state, ordering)

#     # Use automatic differentiation to compute the gradient
#     gradient!(g, params) = g .= ForwardDiff.gradient(objective, params)

#     # Initial guess for parameters
#     initial_params = [0.001, 0.01]  # initial slop and damping

#     # Optimize using LBFGS
#     result = optimize(objective, gradient!, initial_params, LBFGS())

#     return Optim.minimizer(result)
# end

# # Example usage
# # real_data = #rand(61, 10, 2)  # 61 timesteps, 10 nodes, 2 dimensions
# initial_state = -9pi/10#rand(10, 3)  # 10 nodes, 3 dimensions (including orientation)

# optimal_params = optimize_parameters(real_data, simulated_data, initial_state)
# println("Optimal parameters: slop = $(optimal_params[1]), damping = $(optimal_params[2])")

# # Visualize results
# optimal_simulated_data = simulate_mechanism(optimal_params..., initial_state, size(real_data, 1))
# # Use your existing animation function to visualize the results
# # animate_mechanism_movement(real_data, optimal_simulated_data, ordering)
function set_dampers!(joints, value::Real)
    for joint in joints
        (value==0) && break
        typeof(joint) <: JointConstraint{T,0} where T && continue # floating base
        joint.damper = true
        joint.translational.damper=value
        joint.rotational.damper=value
    end
end

function set_dampers!(joints, dampers::AbstractArray)
    for (i,(joint, value)) in enumerate(zip(joints, dampers))
        (value==0) && continue
        typeof(joints[i]) <: JointConstraint{T,0} where T && continue # floating base
        joint.damper = true
        joint.translational.damper=value
        joint.rotational.damper=value
    end
end

function set_limits(mechanism::Mechanism{T}, joint_limits) where T
    joints = JointConstraint{T}[deepcopy(mechanism.joints)...]

    for (joint_symbol,limits) in joint_limits 
        joint = get_joint(mechanism, joint_symbol)
        if input_dimension(joint.translational) == 0 && input_dimension(joint.rotational) == 1
            joints[joint.id] = add_limits(mechanism, joint, 
                rot_limits=[SVector{1}(limits[1]), SVector{1}(limits[2])])
        elseif input_dimension(joint.translational) == 1 && input_dimension(joint.rotational) == 0
            joints[joint.id] = add_limits(mechanism, joint,
                tra_limits=[SVector{1}(limits[1]), SVector{1}(limits[2])])
        else
            @warn "joint limits can only be set for one-dimensional joints"
        end
    end

    return joints
end

function set_slop(mechanism::Mechanism{T}, joint_slops) where T
    joints = JointConstraint{T}[deepcopy(mechanism.joints)...]

    for (joint, slop) in zip(joints, joint_slops)
        if joint.type == Revolute
            joints[joint.id] = Dojo.add_limits(mechanism, joint)
            
        else
            joints[joint.id] = Dojo.add_limits(mechanism, joint,
                tra_limits=[[-slop/2, -slop/2], [slop/2, slop/2]])
        end
    end

    return Mechanism(mechanism.origin, mechanism.bodies, joints;
    mechanism.gravity, mechanism.timestep, mechanism.input_scaling)
end

function step_svd!(mechanism::Mechanism{T}, z::Vector{T}, u::Vector{T}; 
    opts=SolverOptions{T}()) where T
    
    # set state
    set_maximal_state!(mechanism, z)

    # set control
    set_input!(mechanism, u)

    # solve the 1-step simulation problem
    Dojo.mehrotra_svd!(mechanism, opts=opts)
    for body in mechanism.bodies 
        Dojo.update_state!(body, mechanism.timestep) 
    end

    # extract the next state
    z_next = Dojo.get_maximal_state(mechanism)
    # z_next = Dojo.get_next_state(mechanism)
    
    return z_next
end
function get_real_data()
    mechanism = get_scissor_mechanism(num_sets=3, initial_angle=-pi*9/10, damper=0.001, slop=0.001, gravity=[0.0, 0.0, 9.81])
    steps = 60
    real_data = Storage(steps, length(mechanism.bodies))
    simulate!(mechanism, 1:steps, real_data, record=true, opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-7), abort_upon_failure=true, solver=Dojo.mehrotra_svd!)

    return real_data
    # vis = Visualizer()
    # vis = visualize(mechanism, real_data, vis=vis, visualize_floor=false)
end
function func(mechanism, state, dampers, joint_limits)
    opts = SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-7)
    set_dampers!(mechanism.joints, dampers)
    # mechanism = set_slop(mechanism, joint_limits)
    next_state = step_svd!(mechanism, state, zeros(input_dimension(mechanism)), opts=opts)
    return next_state
end


real_data = get_real_data();
mechanism = get_scissor_mechanism(num_sets=3, initial_angle=-pi*9/10, damper=0.001, slop=0.001, gravity=[0.0, 0.0, 9.81])
state = get_maximal_state(mechanism)
damper = [0.01 for i in 1:length(mechanism.joints)] #! find joint damping that best matches data
# limits = 0.001*ones(length(mechanism.joints)) #! Get limits from video kinematics data
out = func(mechanism, state, damper, [])
using FiniteDiff
jac = FiniteDiff.finite_difference_jacobian(x -> func(mechanism, state, x, limits), damper)

using Optim
using LinearAlgebra

# ... (Previous code remains the same)

function objective_function(dampers, mechanism, real_data)
    total_error = 0.0
    max_error = 0.0
    current_state = get_maximal_state(mechanism)
    println("Current dampers: ", dampers)
    for i in 1:size(real_data.x[1])[1]-1
        # println("Step: ", i)
        next_state = func(mechanism, current_state, dampers, [])  # We're not using joint_limits here
        inds = vcat([collect(range) for range in [13 * (i-1) .+ (1:3) for i in 1:length(mechanism.bodies)]]...)
        next_state_x = next_state[inds]
        real_next_state = vcat([collect(x[i+1]) for x in real_data.x]...)
        
        # Calculate error (you might want to adjust this based on your specific needs)
        error = norm(next_state_x - real_next_state)
        # println(error)
        total_error += error
        max_error = max(max_error, error)

        
        current_state = next_state
    end
    
    return total_error, max_error
end

function optimize_dampers(mechanism, real_data)
    n_joints = length(mechanism.joints)
    initial_dampers = [0.001 for _ in 1:n_joints]
    
     # Define the optimization function
     function f(dampers)
        total_error, max_error = objective_function(dampers, mechanism, real_data)
        return total_error
    end
    
    # Custom callback function to check max error
    max_error_tol = 1e-4
    iter = 0
    callback = function(state)
        iter += 1
        println(state[1].metadata["x"])
        _, max_error = objective_function(state[1].metadata["x"], mechanism, real_data)
        if iter % 10 == 0  # Print every 10 iterations
            println("Iteration $iter: Max error = $max_error")
        end
        return max_error <= max_error_tol
    end
    
    # Set up optimization options
    options = Optim.Options(
        iterations = 10000,  # Increased max iterations
        store_trace = true,
        show_trace = false,  # Set to false to avoid cluttering output
        callback = callback,
        extended_trace = true
    )
    
    # Define lower and upper bounds for dampers
    lower = zeros(n_joints)  # All dampers must be >= 0.0
    upper = fill(Inf, n_joints)  # No upper limit
    
    # Run the optimization using L-BFGS-B with box constraints
    result = optimize(f, lower, upper, initial_dampers, Fminbox(GradientDescent(linesearch=LineSearches.BackTracking(order=3))), options)
    
    # Extract the optimized dampers
    optimized_dampers = Optim.minimizer(result)
    
    return optimized_dampers, result
end
using LineSearches

# Run the optimization
# mechanism, real_data = get_real_data()
real_data = get_real_data();
mechanism = get_scissor_mechanism(num_sets=3, initial_angle=-pi*9/10, damper=0.001, slop=0.001, gravity=[0.0, 0.0, 9.81]);
# z = get_maximal_state(mechanism)
# opts = SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-7)
# out = step_svd!(mechanism, z, zeros(input_dimension(mechanism)), opts=opts)
# set_maximal_state!(mechanism, z)
# sim_storage = Storage(2, length(mechanism.bodies))
# simulate!(mechanism, 1:2, sim_storage, record=true, opts=opts, abort_upon_failure=true, solver=Dojo.mehrotra_svd!)



dampers = [0.001 for i in 1:length(mechanism.joints)]
# current_state = get_maximal_state(mechanism)
# next_state = func(mechanism, current_state, dampers, [])  # We're 

# # objective_function(dampers, mechanism, real_data)
# delete!(vis)
# vis = visualize(mechanism, real_data, vis=vis, visualize_floor=false)
optimized_dampers, optimization_result = optimize_dampers(mechanism, real_data);
println(optimization_result)

# Print results
println("Optimized dampers: ", optimized_dampers)
println("Final objective value: ", Optim.minimum(optimization_result))
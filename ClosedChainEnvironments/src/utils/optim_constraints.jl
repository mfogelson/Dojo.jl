using Optim
using LinearAlgebra
using Random
using Dojo

# Define a function to convert a quaternion to a rotation matrix
function quaternion_to_rotation_matrix(q)
    q0, q1, q2, q3 = q
    [
        1-2(q2^2 + q3^2) 2(q1*q2 - q0*q3) 2(q1*q3 + q0*q2)
        2(q1*q2 + q0*q3) 1-2(q1^2 + q3^2) 2(q2*q3 - q0*q1)
        2(q1*q3 - q0*q2) 2(q2*q3 + q0*q1) 1-2(q1^2 + q2^2)
    ]
end

# Define the objective function for non-linear least squares
# function objective_function(q, P, Q)
#     R = quaternion_to_rotation_matrix(q)
#     sum(norm(Q[:, i] - R * P[:, i])^2 for i in 1:size(P, 2))
# end

function objective_function(z, mechanism)
    # R = quaternion_to_rotation_matrix(q)
    set_maximal_state!(mechanism, z)
    for body in mechanism.bodies
        body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
        # println(Dojo.norm(body.state.q2))
    end
    Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2
    res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))

    return 0.5 * dot(res, res)
    # sum(norm(Q[:, i] - R * P[:, i])^2 for i in 1:size(P, 2))
end

# Generate some random data points
Random.seed!(42)

# Define the mechanism
mechanism = initialize_mechanism(3)

# Initialize the visualization
vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false)
path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
mechanism = Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
z = get_maximal_state(mechanism)
z = randn(length(z))
set_maximal_state!(mechanism, z)
zero_velocities!(mechanism)
for body in mechanism.bodies
    body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
    body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
    println(Dojo.norm(body.state.q2))
end
Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false)

Dojo.step!(mechanism, get_maximal_state(mechanism), zeros(input_dimension(mechanism)))
zero_velocities!(mechanism)

# P = rand(3, 10)
# Apply a known rotation to P to generate Q
# true_q = normalize([1.0, 0.5, 0.5, 0.5])
# R = quaternion_to_rotation_matrix(true_q)
# Q = R * P

# Initial guess for the quaternion
# initial_q = normalize([1.0, 0.0, 0.0, 0.0])
initial_z = get_maximal_state(mechanism)
# Optimize using the Nelder-Mead method
result = optimize(z -> objective_function(z, mechanism), initial_z, NelderMead(), Optim.Options(iterations=50000))

# Extract the optimized quaternion
optimized_z = result.minimizer
set_maximal_state!(mechanism, optimized_z)
for body in mechanism.bodies
    body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
    body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
    println(Dojo.norm(body.state.q2))
end
Dojo.initialize_state!(mechanism) 
# vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false)

res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
maximum(abs.(res))
minimum(abs.(res))
# Print the results
println("True quaternion: $true_q")
println("Optimized quaternion: $optimized_q")
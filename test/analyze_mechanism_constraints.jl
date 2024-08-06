using Dojo
using JLD2
using Plots

include("initialize_constraints_refactor.jl")
include("make_jansen.jl")
include("make_scissor.jl")



function analyze_data(mechanism, storage)
    # for each step calculate the norm of the residual of the constraints, get the jacobian and the singular values and store these for future plotting 
    N = length(storage.x[1])
    residuals = zeros(N)
    singular_values = []
    jacobians = []

    fixedids = get_fixed_ids(mechanism)
    freebodies, freeids = get_free_bodies(mechanism, fixedids = fixedids)

    jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freeids)

    function load_state_from_storage(mechanism, storage, step)
        for (i, body) in enumerate(mechanism.bodies)
            body.state.x2 = storage.x[i][step]
            body.state.q2 = storage.q[i][step]
        end

    end


    for i = 1:N
        load_state_from_storage(mechanism, storage, i)

        z = get_maximal_configuration(mechanism, freeids)

        residuals[i] = norm(get_residual(mechanism, freeids, z))

        update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx)

        push!(jacobians, deepcopy(jac))

        F = svd(jac, full=true, alg=LinearAlgebra.QRIteration())

        push!(singular_values, deepcopy(F))

    end
    return residuals, singular_values
end

function plot_singular_distribution(singular_values)
    # plot the maximum and minimum singular values
    max_singular_values = [maximum(s.S) for s in singular_values]
    min_singular_values = [minimum(s.S) for s in singular_values]
    plot(max_singular_values, label="max singular values")
    plot!(min_singular_values, label="min singular values", ylabel="Singular Values")

end

function plot_residuals(residuals)
    plot(residuals, label="residuals", ylabel="Residuals", color=:green, legend=:bottomright)
end

function plot_num_singular_values(singular_values)
    num_singular_values = [sum(s.S .> 1e-6) for s in singular_values]
    full_rank = [length(s.S) for s in singular_values]
    plot(num_singular_values, label="num singular values > 1e-6", ylabel="Num Singular Values > 1e-6")
    plot!(full_rank, label="full rank")
end


# ============================================================================ #
# Analyze Jansen

# Load the mechanism
mechanism = make_jansen_full()

# Load the data 
storage = load("jansen_full_old.jld2")["storage"]

# Analyze the data
residuals, singular_values = analyze_data(mechanism, storage)

# Plot the results
plot_residuals(residuals)
plot_num_singular_values(singular_values)
plot_singular_distribution(singular_values)
# ============================================================================ #



# ============================================================================ #
# Analyze Scissor Cells
all_residuals = []
all_singular_values = []
for i in 2:10
    mechanism = load("scissor_cells_$(i).jld2")["mechanism"]
    storage = load("scissor_cells_$(i).jld2")["storage"]

    # Analyze the data
    residuals, singular_values = analyze_data(mechanism, storage)

    push!(all_residuals, residuals)
    push!(all_singular_values, singular_values)
end

plot(all_residuals[1], label="residuals 2_cells", ylabel="Residuals", color=:green, legend=:bottomright)

for i in 3:10
    plot!(all_residuals[i-1], label="residuals $(i)_cells")
end
plot!()

min_singular_values = [sum(s.S .< 1e-6) for s in all_singular_values[1]]
plot(min_singular_values, label="min singular values 2_cells", ylabel="Num Singular Values < 1e-6", color=:green, legend=:bottomright)
for i in 3:10
    min_singular_values = [sum(s.S .< 1e-6) for s in all_singular_values[i-1]]
    plot!(min_singular_values, label="min singular values $(i)_cells")
end
plot!()
# ============================================================================ #


# ============================================================================ #
# Analyze PET Data 
all_residuals = []
all_singular_values = []
for i in 1:10
    mechanism = load("PET_1.9_$(i)cells.jld2")["mechanism"]
    storage = load("PET_1.9_$(i)cells.jld2")["storage"]

    # Analyze the data
    residuals, singular_values = analyze_data(mechanism, storage)

    push!(all_residuals, residuals)
    push!(all_singular_values, singular_values)
end

plot(all_residuals[1][1:50], label="residuals 1_cells", ylabel="Residuals", color=:green, legend=:topright)

for i in 2:10
    plot!(all_residuals[i][1:50], label="residuals $(i)_cells")
end
plot!()

min_singular_values = [sum(s.S .< 1e-6) for s in all_singular_values[1]]
plot(min_singular_values, label="min singular values 1cell", ylabel="Num Singular Values < 1e-6", color=:green, legend=:topright)
for i in 2:10
    min_singular_values = [sum(s.S .< 1e-6) for s in all_singular_values[i]]
    plot!(min_singular_values, label="min singular values $(i)_cells")
end
plot!()

mechanism = load("PET_1.9_3cells.jld2")["mechanism"]
storage = load("PET_1.9_3cells.jld2")["storage"]
F = all_singular_values[end][1]

delete!(vis)
visualize(mechanism, storage, vis=vis, visualize_floor=false)


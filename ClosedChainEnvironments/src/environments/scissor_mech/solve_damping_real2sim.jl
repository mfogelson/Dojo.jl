using Dojo
using Pkg
Pkg.activate("./DojoEnvironments")
using DojoEnvironments
Pkg.activate(".")
using LinearAlgebra
using Optim
include("scissor_mechanism.jl")
include("controllers.jl")

# Compare real and simulated data
# Function to sort by X-coordinate within Z-based groupings
function sort_by_x_within_z_sets(real_points, sim_points)
    # Group points into top, middle, and bottom based on Z values
    function group_points_by_z(points)
        top_points = [points[i, :] for i in 1:size(points)[1] if points[i, :][2] > 0.02]  # Top set
        middle_points = [points[i, :] for i in 1:size(points)[1] if abs(points[i, :][2]) ≤ 0.02]  # Middle set
        bottom_points = [points[i, :] for i in 1:size(points)[1] if points[i, :][2] < -0.02]  # Bottom set
        
        return top_points, middle_points, bottom_points
    end
    
    # Sort points in each group by X-coordinate
    function sort_by_x(points)
        return sort(points, by = x -> x[1])  # Sort by X (first coordinate)
    end

    # Group and sort real points
    real_top, real_middle, real_bottom = group_points_by_z(real_points)
    sorted_real_top = sort_by_x(real_top)
    sorted_real_middle = sort_by_x(real_middle)
    sorted_real_bottom = sort_by_x(real_bottom)
    
    # Group and sort simulated points
    sim_top, sim_middle, sim_bottom = group_points_by_z(sim_points)
    sorted_sim_top = sort_by_x(sim_top)
    sorted_sim_middle = sort_by_x(sim_middle)
    sorted_sim_bottom = sort_by_x(sim_bottom)
    
    # Combine sorted sets back together
    sorted_real_points = vcat(sorted_real_top, sorted_real_middle, sorted_real_bottom)
    sorted_sim_points = vcat(sorted_sim_top, sorted_sim_middle, sorted_sim_bottom)

    return sorted_real_points, sorted_sim_points
end

function sort_by_x_within_z_sets(real_points, sim_points)
    # Group points into top, middle, and bottom based on Z values and return indices
    function group_points_by_z(points)
        top_inds = [i for i in 1:size(points, 1) if points[i, 2] > 0.02]  # Top set (based on Z)
        middle_inds = [i for i in 1:size(points, 1) if abs(points[i, 2]) ≤ 0.02]  # Middle set
        bottom_inds = [i for i in 1:size(points, 1) if points[i, 2] < -0.02]  # Bottom set
        
        return top_inds, middle_inds, bottom_inds
    end
    
    # Sort indices in each group by X-coordinate
    function sort_by_x_inds(points, inds)
        return sort(inds, by = i -> points[i, 1])  # Sort indices by the X-coordinate
    end

    # Group and sort real points, but return indices
    real_top_inds, real_middle_inds, real_bottom_inds = group_points_by_z(real_points)
    sorted_real_top_inds = sort_by_x_inds(real_points, real_top_inds)
    sorted_real_middle_inds = sort_by_x_inds(real_points, real_middle_inds)
    sorted_real_bottom_inds = sort_by_x_inds(real_points, real_bottom_inds)
    
    # Group and sort simulated points, but return indices
    sim_top_inds, sim_middle_inds, sim_bottom_inds = group_points_by_z(sim_points)
    sorted_sim_top_inds = sort_by_x_inds(sim_points, sim_top_inds)
    sorted_sim_middle_inds = sort_by_x_inds(sim_points, sim_middle_inds)
    sorted_sim_bottom_inds = sort_by_x_inds(sim_points, sim_bottom_inds)
    
    # Combine sorted indices back together for real and sim points
    sorted_real_inds = vcat(sorted_real_top_inds, sorted_real_middle_inds, sorted_real_bottom_inds)
    sorted_sim_inds = vcat(sorted_sim_top_inds, sorted_sim_middle_inds, sorted_sim_bottom_inds)

    return sorted_real_inds, sorted_sim_inds
end


# Extract positions from state trajectory
function extract_positions(trajectory)
    positions = zeros(Float64, length(trajectory.x[1]), 3*convert(Int, length(trajectory.x)/2)-1, 2)
    for j in 1:length(trajectory.x[1])
        for i in 1:convert(Int, 23)
            center = trajectory.x[2*i-1][j]
            top_right = trajectory.x[2*i-1][j] + Dojo.rotation_matrix(trajectory.q[2*i-1][j]) * [0, 0, 0.045/2]
            top_left = trajectory.x[2*i-1][j] + Dojo.rotation_matrix(trajectory.q[2*i-1][j]) * [0, 0, -0.045/2]
            bottom_right = trajectory.x[2*i][j] + Dojo.rotation_matrix(trajectory.q[2*i][j]) * [0, 0, 0.045/2]
            bottom_left = trajectory.x[2*i][j] + Dojo.rotation_matrix(trajectory.q[2*i][j]) * [0, 0, -0.045/2]
            if i == 1
                positions[j, 1, :] = [center[3], center[2]]
                positions[j, 2, :] = [top_right[3], top_right[2]]
                # positions[j, 3, :] = [top_left[3], top_left[2]]
                positions[j, 3, :] = [bottom_right[3], bottom_right[2]]
                positions[j, 4, :] = [bottom_left[3], bottom_left[2]]
            elseif i == 23
                positions[j, end, :] = [center[3], center[2]]
            else           
                positions[j, 3*(i-1)+1+1, :] = [center[3], center[2]]
                positions[j, 3*(i-1)+2+1, :] = [top_right[3], top_right[2]]
                positions[j, 3*(i-1)+3+1, :] = [bottom_right[3], bottom_right[2]]
            end
        end
    end
    return positions  # Assuming first 3 elements are positions
end

# Compute residuals between simulated and target trajectories
function compute_residuals(damping, angle, target_positions, duration, num_steps)
    # mech = create_snake(dampers=damping)
    # simulated_trajectory = simulate_snake(mech, duration, num_steps)
    mech = get_scissor_mechanism(num_sets=23, damper=damping, initial_angle=angle, timestep=duration/num_steps, slop=0.00089)
    simulated_trajectory = Storage(num_steps, length(mech.bodies))
    simulated_trajectory = simulate!(mech, 1:num_steps, simulated_trajectory, spring_controller!, 
            record=true, 
            opts=SolverOptions(rtol=1e-5, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-6),
            abort_upon_failure=true,
            solver=Dojo.mehrotra_svd!)

    simulated_positions = extract_positions(simulated_trajectory)
    # target_positions = extract_positions(target_trajectory)
    
    #TODO fix this so that it works with the scissor mechanism real data
    # Sort real and simulated data
    sorted_real_data_ind, sorted_simulated_data_ind = sort_by_x_within_z_sets(target_positions[1, :, :].-reshape(target_positions[1,1,:], 1, 2), simulated_positions[1, :, :])
    sorted_real_data = target_positions[:, sorted_real_data_ind, :]
    sorted_simulated_data = simulated_positions[:, sorted_simulated_data_ind, :]
    #flatten the data
    residuals = sorted_simulated_data - sorted_real_data
    residuals = reshape(residuals, size(residuals)[1], size(residuals)[2]*size(residuals)[3])

    # residuals = vcat([sim - target for (sim, target) in zip(simulated_positions, target_positions)]...)
    println("Residuals: ", dot(residuals, residuals))
    return residuals
end

# Objective function for optimization
function objective(damping_angle, target_trajectory, duration, num_steps)
    residuals = compute_residuals(damping_angle[1], damping_angle[2], target_trajectory, duration, num_steps)
    return dot(residuals, residuals)
end

# Function to estimate damping using LBFGS
# function estimate_damping(target_trajectory, duration, num_steps; initial_damping=0.1)
#     obj = (damping) -> objective(damping, target_trajectory, duration, num_steps)
#     result = optimize(obj, [initial_damping], LBFGS(), Optim.Options(show_trace = true))
#     return Optim.minimizer(result)[1]
# end
function estimate_damping(target_trajectory, duration, num_steps; initial_damping=0.1, initial_angle=-pi*9/10)
    obj = (damping_angle) -> objective(damping_angle, target_trajectory, duration, num_steps)
    
    # Set up custom stopping criteria
    options = Optim.Options(show_trace = true, g_tol = 1e-4, x_tol = 1e-6, f_tol = 1e-6)
    
    result = optimize(obj, [initial_damping, initial_angle], LBFGS(), options)
    return result, Optim.minimizer(result)[1]
end

# Load and process real data
data_filepath = "/Users/mitchfogelson/Projects/Research_Projects/co-tracker/paper_data/video_1/csv/pred_tracks_formatted.csv"
real_data = load_real_data(data_filepath)
steps = size(real_data)[1]
timestep = 1.0/240
frame = 1
#plot real data and label the scatter points
scatter(real_data[frame, 1:10, 1].-real_data[frame,1,1], real_data[frame, 1:10, 2].-real_data[frame,1,2], 
        label="Real", xlabel="x (m)", ylabel="z (m)", 
        title="Scissor Mechanism Position - Frame 1", 
        legend=:topleft, aspect_ratio=:equal,
        markersize=6, color=:blue)

# Estimate damping
estimated_damping = estimate_damping(real_data, steps*timestep, steps, initial_damping=0.001)

mech = get_scissor_mechanism(num_sets=23, damper=0.01, initial_angle=-2.4824251646528257, timestep=timestep)
simulated_trajectory = Storage(steps, length(mech.bodies))
simulated_trajectory = simulate!(mech, 1:steps, simulated_trajectory, spring_controller!, 
        record=true, 
        opts=SolverOptions(rtol=1e-5, btol=1e-4, reg=1e-8, verbose=false, svd_threshold=1e-6),
        abort_upon_failure=true,
        solver=Dojo.mehrotra_svd!)

simulated_positions = extract_positions(simulated_trajectory)

scatter!(simulated_positions[frame, :, 1].-simulated_positions[frame,1,1], simulated_positions[frame, :, 2].-simulated_positions[frame,1,2], 
        label="sim", xlabel="x (m)", ylabel="z (m)", 
        title="Scissor Mechanism Position - Frame 1", 
        legend=:topleft, aspect_ratio=:equal,
        markersize=6, color=:red)


# Sort real and simulated data
sorted_real_data_inds, sorted_simulated_data_inds = sort_by_x_within_z_sets(real_data[frame, :, :].-reshape(real_data[frame,1,:], 1, 2), simulated_positions[frame, :, :])

sorted_real_data = real_data[:, sorted_real_data_inds, :]
sorted_simulated_data = simulated_positions[:, sorted_simulated_data_inds, :]


# Plot real and simulated data
scatter(sorted_real_data[1,end-10:end,1], sorted_real_data[1,end-10:end,2], label="Real", xlabel="x (m)", ylabel="z (m)", 
        title="Scissor Mechanism Position - Frame 1", 
        legend=:topleft, aspect_ratio=:equal,
        markersize=6, color=:blue)
scatter!([d[1] for d in sorted_simulated_data[1:5]], [d[2] for d in sorted_simulated_data[1:5]], label="Simulated", xlabel="x (m)", ylabel="z (m)",)
# plot the connections 
# for i in 1:22
#     plot!([sorted_real_data[i, 1], sorted_real_data[i+1, 1]], [sorted_real_data[i, 2], sorted_real_data[i+1, 2]], color=:blue)
#     plot!([sorted_simulated_data[i, 1], sorted_simulated_data[i+1, 1]], [sorted_simulated_data[i, 2], sorted_simulated_data[i+1, 2]], color=:red)
# end
errors = sorted_real_data - sorted_simulated_data
# plot errors
scatter(errors[:,:,1], errors[:,:,2], label="", xlabel="x (m)", ylabel="z (m)", 
        title="Scissor Mechanism Position - Frame 1", 
        legend=:topleft, aspect_ratio=:equal,
        markersize=6, color=:green)
using Dojo
using CSV
using DataFrames
using Plots

include("scissor_mechanism.jl")
include("controllers.jl")

"""
Run a single simulation of the scissor mechanism and compare to real data.

Parameters:
- num_cells: Number of scissor units
- slop: Amount of play in the joints
- initial_angle: Initial angle of the mechanism
- steps: Number of simulation steps
- controller: Controller function to use (angle_controller! or spring_controller!)
"""
function run_simulation(num_cells, slop, initial_angle, steps, controller)
    mechanism = get_scissor_mechanism(damper=0.01, num_sets=num_cells, initial_angle=initial_angle, slop=slop, gravity=[0.0, -9.81, 0])
    storage = Storage(steps, length(mechanism.bodies))
    
    simulate!(mechanism, 1:steps, storage, controller, 
              record=true, 
              opts=SolverOptions(rtol=1e-5, btol=1e-4, reg=1e-8, verbose=true, svd_threshold=1e-6),
              abort_upon_failure=true,
              solver=Dojo.mehrotra_svd!)

    return mechanism, storage
end

"""
Load and process real data from CSV file.
"""
function load_real_data(filepath)
    body_pos_df = CSV.read(filepath, DataFrame)
    body_pos_df = select(body_pos_df, Not(:Frame))

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

    return body_pos_meters_real
end

"""
Parse coordinate string for real data.
"""
function parse_coord_real(s::AbstractString)
    x, y = split(replace(s, r"[()]" => ""), ",")
    return parse(Float64, x), parse(Float64, y)
end

"""
Create animation comparing simulated and real data.
"""
function create_animation(simulated_data, real_data, output_path)
    anim = @animate for i in 1:size(real_data, 1)
        scatter(simulated_data[4*i, :, 3], simulated_data[4*i, :, 2], 
                label="Simulated", markersize=7, color=:red)
        scatter!(real_data[i, :, 1]*5.5 .-0.01, real_data[i, :, 2]*5.5 .- 0.07, 
                label="Real", xlabel="x (m)", ylabel="z (m)", 
                title="Scissor Mechanism Position - Frame $i", 
                legend=:topleft, aspect_ratio=:equal,
                markersize=6, color=:blue)
        
        xlims!(-0.03, .3)
        ylims!(-0.06, 0.06)
    end

    gif(anim, output_path, fps = 30)
end

# Example usage
num_cells = 23
slop = 0.00089 #! from real data
initial_angle = 2.4824251646528257 #! from real data #-pi*9/10
steps = 244

mechanism, storage = run_simulation(num_cells, slop, initial_angle, steps, spring_controller!);

vis = Visualizer()
delete!(vis)
vis = visualize(mechanism, storage, vis=vis, show_frame=true, visualize_floor=false, show_joint=true, joint_radius=0.001)

using JLD2
save("paper_data/Scissor_jamming/09_25_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism_gravity_y.jld2", "mechanism", mechanism, "storage", storage)

out = load("paper_data/Scissor_jamming/09_25_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism.jld2")
mechanism = out["mechanism"]
storage = out["storage"]
# Process simulated data
simulated_data = zeros(Float64, steps, length(mechanism.bodies)*3, 3)
for step in 1:steps
    for body in 1:length(mechanism.bodies)
        simulated_data[step, 3*body-2, :] = storage.x[body][step] + Dojo.rotation_matrix(storage.q[body][step]) * [0, 0, -0.045/2]
        simulated_data[step, 3*body-1, :] = storage.x[body][step] + Dojo.rotation_matrix(storage.q[body][step]) * [0, 0, 0.045/2]
        simulated_data[step, 3*body, :] = storage.x[body][step] 
    end
end

# Load and process real data
real_data = load_real_data("/Users/mitchfogelson/Projects/Research_Projects/co-tracker/videos/pred_tracks_formatted.csv")

#plot real data and label the scatter points
scatter(real_data[end, :, 1]*5.5 .-0.01, real_data[end, :, 2]*5.5 .- 0.07, 
        label="Real", xlabel="x (m)", ylabel="z (m)", 
        title="Scissor Mechanism Position - Frame 1", 
        legend=:topleft, aspect_ratio=:equal,
        markersize=6, color=:blue)

# Create animation
create_animation(simulated_data, real_data, "scissor_mechanism_animation.gif")

real_data
simulated_data
v1 = [real_data[1, i,:]*5.5 - [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
v2 = [[simulated_data[1, i,3], simulated_data[1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]

function min_distance(v1, v2)
    # Ensure v1 is the shorter vector
    if length(v1) > length(v2)
        v1, v2 = v2, v1
    end

    min_dist = Inf
    min_alignment = nothing
    
    for i in 1:(length(v2) - length(v1) + 1)
        dist = 0.0
        alignment = Tuple[]
        
        for j in 1:length(v1)
            dist += Dojo.norm(v1[j] - v2[i+j-1])
            push!(alignment, (v1[j], v2[i+j-1]))
        end
        
        dist = sqrt(dist)
        
        if dist < min_dist
            min_dist = dist
            min_alignment = alignment
        end
    end

    return min_dist, min_alignment
end

using LinearAlgebra
using Distances
using Hungarian

function distance_func(curve_i::Matrix{Float64}, curve_j::Matrix{Float64}; ordered::Bool=false, distance_metric::String="euclidean")
    """
    Calculate distance between curve_i and curve_j
    Args:
        curve_i (Matrix{Float64}): Array of x,y points from curve_i. Shape: (n, 2)
        curve_j (Matrix{Float64}): Array of x,y points from curve_j. Shape: (n, 2)
        ordered (Bool): if ordering of points matters. Defaults to false.
        distance_metric (String): metric for distance calculation. Defaults to "euclidean".
    Returns:
        Vector{Float64}: Distance between matched points
    """
    # Correct Shape
    curve_i = size(curve_i, 2) != 2 ? curve_i' : curve_i
    curve_j = size(curve_j, 2) != 2 ? curve_j' : curve_j
    
    # Get distance between all sets of points
    dist_func = getfield(Distances, Symbol(distance_metric))
    C = [dist_func(curve_i[i,:], curve_j[j,:]) for i in 1:size(curve_i,1), j in 1:size(curve_j,1)]
    
    row_ind = collect(1:size(curve_i,1))
    
    if ordered
        row_inds = [row_ind for _ in 1:length(row_ind)]
        col_inds = [circshift(row_ind, -i) for i in 0:length(row_ind)-1]
        
        sum_distances = [sum(C[row_inds[i], col_inds[i]]) for i in 1:length(row_ind)]
        min_clock_wise, argmin_clock_wise = findmin(sum_distances)
        
        sum_distances_reverse = [sum(C[row_inds[i], reverse(col_inds[i])]) for i in 1:length(row_ind)]
        min_count_clock_wise, argmin_count_clock_wise = findmin(sum_distances_reverse)
        
        # Check both directions of ordering
        cw_dir = Int(min_clock_wise < min_count_clock_wise)
        col_ind = cw_dir * col_inds[argmin_clock_wise] + (1-cw_dir) * reverse(col_inds[argmin_count_clock_wise])
    else   
        col_ind = hungarian(C)[1]
        # col_ind = [findfirst(==(i), assignment) for i in 1:length(assignment)]
    end
    
    return col_ind, [C[i, col_ind[i]] for i in 1:length(col_ind)]
end


ordering, dist_vect = distance_func( hcat(hcat(v1)...),  hcat(hcat(v2)...), ordered=false)

alignment = [(v1[i], v2[ordering[i]]) for i in 1:length(v1)]

# # Example usage
# v1 = [0.0, 1.0, 2.0]
# v2 = [0.0, 2.0, 3.0, 1.0, 4.0]

# distance, alignment = min_distance(v1, v2)
using Plots
scatter([v1[i][1] for i in 1:length(v1)], [v1[i][2] for i in 1:length(v1)], label="Real", markersize=7, color=:blue)
scatter!([v2[i][1] for i in 1:length(v2)], [v2[i][2] for i in 1:length(v2)], label="Simulated", markersize=7, color=:red)
for (p1, p2) in alignment[2:end]
    plot!([p1[1], p2[1]], [p1[2], p2[2]], color=:black, lw=2, label="")
end
plot!()
println("Minimum distance: ", distance)
println("Alignment: ", alignment)

function has_duplicates(vec)
    return length(vec) != length(Set(vec))
end

function find_duplicates(vec)
    freq = Dict{eltype(vec), Int}()
    for item in vec
        freq[item] = get(freq, item, 0) + 1
    end
    return [k for (k, v) in freq if v > 1]
end

function find_duplicates_with_counts(vec)
    freq = Dict{eltype(vec), Int}()
    for item in vec
        freq[item] = get(freq, item, 0) + 1
    end
    return Dict(k => v for (k, v) in freq if v > 1)
end
find_duplicates_with_counts(ordering)

dists = []
for n in 1:size(real_data, 1)
    v1 = [real_data[n, i,:]*5.5 - [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
    v2 = [[simulated_data[4*(n-1)+1, i,3], simulated_data[4*(n-1)+1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]
    dist = 0.0
    for j in 1:size(real_data, 2)
        dist += norm(v1[j] - v2[ordering[j]])
    end
    push!(dists, dist)
end

plot(dists)


n = 61
v1 = [real_data[n, i,:]*5.5 - [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
v2 = [[simulated_data[4*(n-1)+1, i,3], simulated_data[4*(n-1)+1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]
scatter([v1[i][1] for i in 1:length(v1)], [v1[i][2] for i in 1:length(v1)], label="Real", markersize=7, color=:blue)
scatter!([v2[i][1] for i in 1:length(v2)], [v2[i][2] for i in 1:length(v2)], label="Simulated", markersize=7, color=:red)
for j in 1:length(ordering)
    plot!([v1[j][1], v2[ordering[j]][1]], [v1[j][2], v2[ordering[j]][2]], color=:black, lw=2, label="")
end
plot!() 


function create_frame(n, real_data, simulated_data, ordering)
    v1 = [real_data[n, i,:]*5.5 - [0.01, 0.07] for i in 1:size(real_data[1, :, :])[1]]
    v2 = [[simulated_data[4*(n-1)+1, i,3], simulated_data[4*(n-1)+1, i,2]] for i in 1:size(simulated_data[1, :, :])[1]]
    
    p = scatter([v[1] for v in v1], [v[2] for v in v1], 
                label="Real", markersize=7, color=:blue, 
                xlabel="X", ylabel="Y", title="Scissor Movement Comparison (Frame $n)", aspect_ratio=:equal, xlims=(-0.03, 0.3), ylims=(-0.06, 0.06))
    scatter!(p, [v[1] for v in v2], [v[2] for v in v2], 
             label="Simulated", markersize=7, color=:red)
    
    for j in 1:length(ordering)
        plot!(p, [v1[j][1], v2[ordering[j]][1]], [v1[j][2], v2[ordering[j]][2]], 
              color=:black, lw=2, label="")
    end
    
    return p
end

function animate_fish_movement(real_data, simulated_data, ordering)
    anim = @animate for n in 1:61
        create_frame(n, real_data, simulated_data, ordering)
    end
    
    return gif(anim, "09_26_2024_scissor_movement.gif", fps = 5)
end

# Assuming real_data, simulated_data, and ordering are already defined
# real_data: shape (61, num_fish, 2)
# simulated_data: shape (244, num_fish, 3)
# ordering: vector of indices

# Create the animation
animate_fish_movement(real_data, simulated_data, ordering)

println("Animation saved as 'fish_movement.gif'")
using Plots
using LinearAlgebra
using Statistics
"""
    csv2struct(filename::String)

Converts OptiTrack Motive CSV exports to a structured data format.
Returns a Dict containing marker positions, time data, frame info, and anthropometric data.
"""
function csv2struct(filename::String)
    # Read file data
    file_path = endswith(filename, ".csv") ? filename : filename * ".csv"
    lines = readlines(file_path)

    # Parse header information
    header_lines = 7  # Number of header lines before data
    header = split.(lines[1:header_lines], ",")
    frame_rate = parse(Float64, header[1][8])
    total_frames = parse(Int, header[1][16])

    # Parse marker information
    markers_line = split(lines[4], ",")
    n_markers = length(markers_line) ÷ 3

    # Read raw data starting from line 8
    data_lines = lines[8:end]
    data = Array{Float64}(undef, total_frames, (n_markers*3)+2)

    for (i, line) in enumerate(data_lines)
        if isempty(line) || i > total_frames
            break
        end
        values = split(line, ",")
        for j in 1:min(length(values), size(data, 2))
            value_str = strip(values[j])
            data[i, j] = isempty(value_str) ? NaN : parse(Float64, value_str)
        end
    end

    # Extract time vector
    time_vector = data[:, 2]

    # Create 3D marker array
    markers = Array{Float64}(undef, total_frames, n_markers, 3)
    for f in 1:total_frames
        c = 1
        for x in 3:3:(n_markers*3)+2
            markers[f, c, 1] = data[f, x]             # X remains X
            markers[f, c, 3] = data[f, x+1]           # Y becomes Z
            markers[f, c, 2] = -1.0 * data[f, x+2]    # Z becomes -Y
            c += 1
        end
    end

    # Find markersets
    marker_labels = split.(lines[4:6], ",")
    marker_labels = vcat(marker_labels...)

    # Extract set names
    set_names = String[]
    for label in marker_labels
        if isempty(label) || !contains(label, "Unlabeled")
            continue
        end
        set_name = split(label, ":")[1]
        if !(set_name in set_names)
            push!(set_names, set_name)
        end
    end

    # Count markers per set
    markers_per_set = Dict{String, Int}()
    for set_name in set_names
        count = 0
        for label in marker_labels
            if startswith(label, set_name)
                count += 1
            end
        end
        markers_per_set[set_name] = count ÷ 3  # Divide by 3 since each marker has X, Y, Z
    end

    # Organize markers in structure
    result = Dict{String, Any}(
        "FrameRate" => frame_rate,
        "Time" => time_vector,
        "Frames" => total_frames,
        "Sets" => Dict{String, Any}(),
        "Anthropometry" => Dict{String, Any}()
    )

    label_index = 1
    processed_markers = 0

    for set_name in set_names
        result["Sets"][set_name] = Dict{String, Any}(
            "Raw" => Dict{String, Any}(),
            "NMarkers" => markers_per_set[set_name]
        )
        
        # Get cleaned marker names for this set
        set_markers = String[]
        for i in label_index:label_index+markers_per_set[set_name]-1
            if i <= n_markers
                marker_label = marker_labels[(i-1)*3+1]
                marker_name = replace(marker_label, set_name => "")
                push!(set_markers, marker_name)
            end
        end
        
        # Store marker data
        for (i, marker_name) in enumerate(set_markers)
            marker_idx = processed_markers + i
            if marker_idx <= n_markers
                result["Sets"][set_name]["Raw"][marker_name] = markers[:, marker_idx, :]
            end
        end
        
        processed_markers += markers_per_set[set_name]
        label_index += markers_per_set[set_name]

    end

    return result 
end
    


"""
    plot_optitrack_data(data_dict)

Plots 3D marker positions from OptiTrack data.
- Creates trajectory plots for all markers
- Highlights the fixed reference markers
- Creates an animation of the markers over time
"""
function plot_optitrack_data(data_dict)
    # Extract time information
    time_vector = data_dict["Time"]
    total_frames = data_dict["Frames"]
    
    # Create arrays to store all marker data
    all_markers = []
    marker_names = []
    
    # Extract marker data from all sets
    for (set_name, set_data) in data_dict["Sets"]
        for (marker_name, marker_positions) in set_data["Raw"]
            # Skip markers with no data
            if all(isnan, marker_positions)
                continue
            end
            
            push!(all_markers, marker_positions)
            push!(marker_names, "$(set_name):$(marker_name)")
        end
    end
    
    # Count valid markers
    n_markers = length(all_markers)
    println("Found $(n_markers) markers with valid data")
    
    # Identify reference markers (assuming the first 3 markers are the fixed origins as mentioned)
    reference_markers = all_markers[1:min(3, n_markers)]
    reference_names = marker_names[1:min(3, n_markers)]
    
    # Plot reference markers
    println("Plotting reference markers...")
    p_ref = plot3d(
        title = "Fixed Reference Markers",
        xlabel = "X (m)",
        ylabel = "Y (m)",
        zlabel = "Z (m)",
        legend = true,
        size = (800, 600),
        camera = (30, 30)
    )
    
    colors = [:blue, :red, :green]
    for (i, marker) in enumerate(reference_markers)
        # Filter out NaN values
        valid_indices = findall(row -> !any(isnan, row), eachrow(marker))
        if !isempty(valid_indices)
            # Extract valid X, Y, Z coordinates
            x_vals = [marker[idx, 1] for idx in valid_indices]
            y_vals = [marker[idx, 2] for idx in valid_indices]
            z_vals = [marker[idx, 3] for idx in valid_indices]
            
            # Plot marker trajectory
            plot3d!(p_ref, x_vals, y_vals, z_vals,
                   label = reference_names[i],
                   color = colors[i],
                   linewidth = 2,
                   markersize = 3,
                   markershape = :circle)
        end
    end
    
    # Plot all markers trajectories
    println("Plotting all marker trajectories...")
    p_all = plot3d(
        title = "All Marker Trajectories",
        xlabel = "X (m)",
        ylabel = "Y (m)",
        zlabel = "Z (m)",
        legend = false,
        size = (800, 600),
        camera = (30, 30)
    )
    
    # First plot reference markers with distinct colors
    for (i, marker) in enumerate(reference_markers)
        valid_indices = findall(row -> !any(isnan, row), eachrow(marker))
        if !isempty(valid_indices)
            x_vals = [marker[idx, 1] for idx in valid_indices]
            y_vals = [marker[idx, 2] for idx in valid_indices]
            z_vals = [marker[idx, 3] for idx in valid_indices]
            
            plot3d!(p_all, x_vals, y_vals, z_vals,
                   color = colors[i],
                   linewidth = 2,
                   markersize = 3,
                   markershape = :circle)
        end
    end
    
    # Then plot other markers with transparency
    for i in 4:n_markers
        marker = all_markers[i]
        valid_indices = findall(row -> !any(isnan, row), eachrow(marker))
        
        # Only plot markers with sufficient data points
        if length(valid_indices) > 10
            x_vals = [marker[idx, 1] for idx in valid_indices]
            y_vals = [marker[idx, 2] for idx in valid_indices]
            z_vals = [marker[idx, 3] for idx in valid_indices]
            
            plot3d!(p_all, x_vals, y_vals, z_vals,
                   color = :gray,
                   linewidth = 1,
                   alpha = 0.3,
                   markersize = 1,
                   markershape = :circle)
        end
    end
    
    # Find the most prominent markers (those with most data points)
    marker_data_counts = [count(row -> !any(isnan, row), eachrow(marker)) for marker in all_markers]
    sorted_indices = sortperm(marker_data_counts, rev=true)
    
    # Select top 10 markers by data availability
    top_markers = min(10, n_markers)
    selected_indices = sorted_indices[1:top_markers]
    
    # Plot selected markers
    println("Plotting top $(top_markers) markers by data availability...")
    p_selected = plot3d(
        title = "Top $(top_markers) Marker Trajectories",
        xlabel = "X (m)",
        ylabel = "Y (m)",
        zlabel = "Z (m)",
        legend = true,
        size = (800, 600),
        camera = (30, 30)
    )
    
    marker_colors = [:blue, :red, :green, :purple, :orange, :cyan, :magenta, :yellow, :black, :brown]
    
    for (i, idx) in enumerate(selected_indices)
        marker = all_markers[idx]
        valid_indices = findall(row -> !any(isnan, row), eachrow(marker))
        
        if !isempty(valid_indices)
            x_vals = [marker[idx, 1] for idx in valid_indices]
            y_vals = [marker[idx, 2] for idx in valid_indices]
            z_vals = [marker[idx, 3] for idx in valid_indices]
            
            color_idx = mod(i-1, length(marker_colors)) + 1
            
            plot3d!(p_selected, x_vals, y_vals, z_vals,
                   label = marker_names[idx],
                   color = marker_colors[color_idx],
                   linewidth = 2,
                   markersize = 2,
                   markershape = :circle)
            
            # Print statistics for this marker
            println("Marker $(marker_names[idx]): $(length(valid_indices))/$(total_frames) frames")
            if !isempty(valid_indices)
                println("  X range: $(round(minimum(x_vals), digits=5)) to $(round(maximum(x_vals), digits=5))")
                println("  Y range: $(round(minimum(y_vals), digits=5)) to $(round(maximum(y_vals), digits=5))")
                println("  Z range: $(round(minimum(z_vals), digits=5)) to $(round(maximum(z_vals), digits=5))")
            end
        end
    end
    
    # Create an animation of markers over time
    println("Creating marker animation...")
    
    # For efficiency, sample frames
    sample_rate = max(1, div(total_frames, 100))
    sampled_frames = 1:sample_rate:total_frames
    
    # Find global coordinate bounds for consistent plotting
    x_min, x_max = Inf, -Inf
    y_min, y_max = Inf, -Inf
    z_min, z_max = Inf, -Inf
    
    for marker in all_markers
        valid_indices = findall(row -> !any(isnan, row), eachrow(marker))
        
        if !isempty(valid_indices)
            x_vals = [marker[idx, 1] for idx in valid_indices]
            y_vals = [marker[idx, 2] for idx in valid_indices]
            z_vals = [marker[idx, 3] for idx in valid_indices]
            
            x_min = min(x_min, minimum(x_vals))
            x_max = max(x_max, maximum(x_vals))
            y_min = min(y_min, minimum(y_vals))
            y_max = max(y_max, maximum(y_vals))
            z_min = min(z_min, minimum(z_vals))
            z_max = max(z_max, maximum(z_vals))
        end
    end
    
    # Add a small margin
    margin = 0.05 * max(x_max - x_min, y_max - y_min, z_max - z_min)
    x_min -= margin; x_max += margin
    y_min -= margin; y_max += margin
    z_min -= margin; z_max += margin
    
    # Create animation
    anim = @animate for frame in sampled_frames
        p = plot3d(
            title = "Marker Positions at Frame $(frame) (Time: $(round(time_vector[frame], digits=2)) s)",
            xlabel = "X (m)",
            ylabel = "Y (m)",
            zlabel = "Z (m)",
            xlim = (x_min, x_max),
            ylim = (y_min, y_max),
            zlim = (z_min, z_max),
            legend = false,
            size = (800, 600),
            camera = (30, 30)
        )
        
        # Plot each marker at this frame
        for (i, marker) in enumerate(all_markers)
            if frame <= size(marker, 1) && !any(isnan, marker[frame, :])
                x_val = marker[frame, 1]
                y_val = marker[frame, 2]
                z_val = marker[frame, 3]
                
                # Color: reference markers in their colors, others in gray
                color = i <= 3 ? colors[i] : :gray
                alpha = i <= 3 ? 1.0 : 0.5
                marker_size = i <= 3 ? 5 : 3
                
                scatter3d!(p, [x_val], [y_val], [z_val],
                          color = color,
                          alpha = alpha,
                          markersize = marker_size,
                          markershape = :circle)
            end
        end
    end
    
    # Save plots and animation
    savefig(p_ref, "reference_markers.png")
    savefig(p_all, "all_markers.png")
    savefig(p_selected, "selected_markers.png")
    gif(anim, "marker_animation.gif", fps = 15)
    
    # Return all plots
    return Dict(
        "reference_markers" => p_ref,
        "all_markers" => p_all,
        "selected_markers" => p_selected,
        "animation" => anim
    )
end

# Example usage with the csv2struct function from before
function visualize_optitrack_file(filename::String)
    # First load the data
    data = csv2struct(filename)
    
    # Then visualize it
    plots = plot_optitrack_data(data)
    
    return plots
end

# Usage:
# plots = visualize_optitrack_file("Take 20250503 12.26.42 PM_angulated_scissor.csv")
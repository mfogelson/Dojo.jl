using JLD2, LinearAlgebra, Dojo

# Function to load and process the system data
function process_system(file_path)
    # Load the system data from the .jld2 file
    data = JLD2.jldopen(file_path, "r") do file
        return file["system"] # Adjust this based on the exact structure of your .jld2 file
    end
    
    # Convert the loaded system data to a full matrix
    A = full_matrix(data)
    
    # Perform SVD on the matrix A
    F = svd(A, full=true, alg=LinearAlgebra.QRIteration())

    
    return F
end

# Function to compare singular value matrices to detect changes
function detect_contact_change(singular_values_list)
    # Compare singular values sequentially to detect significant changes
    changes = []
    for i in 1:(length(singular_values_list) - 1)
        # Compute the difference between consecutive singular value vectors
        diff = sum(singular_values_list[i] .< 1e-5) #norm(singular_values_list[i+1] .- singular_values_list[i], Inf)

        push!(changes, diff)
        
        # Define a threshold for detecting significant changes
        if diff > 1e-3 # Adjust this threshold based on your specific application
            println("Significant change detected between system $i and system $(i + 1): diff = $diff")
        end
    end
    return changes
end

# Load and process files sequentially
singular_values_list = []

# Adjust the range based on the number of files you have
for k in 0:2000 # Modify the range as per your files
    file_path = "system$(k).jld2"
    println("Processing: $file_path")
    F = process_system(file_path)
    push!(singular_values_list, F.S) # Store the singular values
end

# Detect changes that could signal contact
changes = detect_contact_change(singular_values_list)
using Plots
plot(changes[2:end], label="Singular Value Changes", xlabel="System Index", ylabel="Change Norm", title="Singular Value Changes Over Time")

# extract orientation of the box at each timestep to see how it changes
# Extract quaternion components
pitches = []
_, storage_0 = load("wedge_block.jld2", "mechanism", "storage")
_, storage_1 = load("wedge_block_501.jld2", "mechanism", "storage")
_, storage_2 = load("wedge_block_1001.jld2", "mechanism", "storage")
_, storage_3 = load("wedge_block_1501.jld2", "mechanism", "storage")
qs = [storage_0.q[1]...; storage_1.q[1]...; storage_2.q[1]...; storage_3.q[1]...]
for q in qs
    w, x, y, z = q.s, q.v1, q.v2, q.v3

    # Convert to Euler angles (yaw, pitch, roll) in radians
    pitch = asin(2.0 * (w * y - z * x))
    push!(pitches, rad2deg(pitch))
end
plot(pitches, label="Pitch", xlabel="Timestep", ylabel="Pitch Angle (radians)", title="Box Pitch Angle Over Time")

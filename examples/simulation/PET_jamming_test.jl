using Dojo

using CSV
using DataFrames
using LinearAlgebra
using JLD2
# Get values from DataFrame
get_param_value(key) = df[findall(isequal(key), df[!, :name]), :value]

# using DataFrames

function convert_to_meters(s::String)
    # Split the string into value and unit
    value_str, unit = split(s)

    # Convert the value to a float
    value = parse(Float64, value_str)

    # Convert the value to meters based on the unit
    if unit == "mm"
        value /= 1000
    elseif unit == "cm"
        value /= 100
    # Add more units as necessary
    end

    return value
end

function get_param_value_in_meters(df::DataFrame, key::String)
    # Extract the parameter value from the DataFrame
    value_str = first(get_param_value(df, key))
    println(value_str)
    
    # Convert the extracted value to meters and return
    return parse(Float64, value_str) #convert_to_meters(value_str)
end

get_param_value(df, key) = df[findall(isequal(key), df[!, :name]), :value]

# Load CSV file 
pathname = "/Users/mitchfogelson/.julia/dev/Dojo.jl/HERDS_design_optimization_final"
filename = joinpath(pathname, "falcon_PET_finalState_mass.csv")
df = CSV.File(filename, header=["name", "units", "value"]) |> DataFrame

thickness = get_param_value_in_meters(df, "thickness")/1000.0
l1 = get_param_value_in_meters(df, "l1")
l2 = get_param_value_in_meters(df, "l2")
l3 = get_param_value_in_meters(df, "l3")
αf = get_param_value_in_meters(df, "alpha")
βf = get_param_value_in_meters(df, "beta")

filename = joinpath(pathname, "falcon_PET_initialState_mass.csv")
df = CSV.File(filename, header=["name", "units", "value"]) |> DataFrame
α0 = get_param_value_in_meters(df, "alpha")
α = [α0, αf]
β0 = get_param_value_in_meters(df, "beta")
β = [β0, βf]
# ind = 2
# ### Parameters
const RADIUS = thickness
const SHORT_LENGTH = 2*l1 
const LONG_LENGTH = l2 + 2*l3
const BOT_LENGTH = 2*l3
const MASS = 0.001 

αs = LinRange(α0, αf, 10)
as = (l2+l3) .* sqrt.(2 .* (1 .- cos.(αs))) # Width 

cs = [Dojo.norm([(l2+l3)*sin(aa/2), (l2+l3)*cos(aa/2)]- ([l3*sin(aa/2), -l3*cos(aa/2)]) - [[cos(pi/2 - aa/2) -sin(pi/2 - aa/2)]; [sin(pi/2 - aa/2) cos(pi/2 - aa/2)]]'*[-l2*sin(pi/2-aa), l2*cos(pi/2-aa)]) for aa in αs]
            
β_check = [-acos(-((c/(2*l1))^2/2-1))-pi + 2pi for c in cs]

d1s = [sqrt((l2+l3)^2+l3^2-2*(l3*(l2+l3))*cos(pi-aa)) for aa in αs]
d2s = [sqrt((l3)^2+l2^2-2*(l3*(l2))*cos(aa)) for aa in αs]

bs = [l1*sqrt(2*(1-cos(bb))) for bb in β_check]  # Edge
# c = 2*l1*sqrt(2*(1-cos(pi-β[ind]))) # Extension
θs =  [acos(max(-1.0, (a^2-2*b^2)/(-2*b^2))) for (a,b) in zip(as, bs)] # cos theta

const PARENT_VERTEX_LONG = [0, 0, -LONG_LENGTH/2]
const PARENT_VERTEX_SHORT = [0, 0, -SHORT_LENGTH/2]
const CHILD_VERTEX_LONG = [0, 0, BOT_LENGTH/2]
const CHILD_VERTEX_SHORT = [0, 0, SHORT_LENGTH/2]
const MIDDLE_VERTEX_LONG = [0, 0, -(LONG_LENGTH - BOT_LENGTH)/2]
const MIDDLE_VERTEX_SHORT = [0, 0, 0.0]
const ROTATION_AXIS = [0, 1, 0]
const L1 = l1
const L2 = l2
const L3 = l3

# Sim params
const DAMPER = 0.0001
const TIMESTEP = 0.001
const REG = 1e-6
const GRAVITY = [0, 0, 0.0]

# System Params
# const NUM_CELLS = 1

const LINE_ITER = 10
const NEWTON_ITER = 400
const DEBUG = false
const EPSILON = 1e-10

function uniquify(filename)
    while isfile(filename)
        split_filename = split(filename, ".")
        if occursin("(", split_filename[1])
            # update the value inside the parentheses by 1
            split_filename[1] = split(split_filename[1], " ")[1]*" ($(parse(Int, split(split_filename[1], " ")[2][2:end-1])+1)"*")"
        else
            # add a new parentheses with value 1
            split_filename[1] = split_filename[1]*" (1)"
        end
        # recombine the filename
        filename = join(split_filename, ".")
    end
    # # recursively call uniquify until unique filename is found
    # uniquify(filename)


    return filename
end

function create_revolute_joint_constraint(parent, child, rotation_axis, parent_vertex, child_vertex, name_ext, i, j, kind; noise=0.0) #2.54e-5)
    # Joint naming convention: joint, cell, link, abbreviated to j, c, and l respectively
    noise_parent = randn(3) * noise
    noise_child = randn(3) * noise
    idx = argmax(rotation_axis)
    noise_parent[idx] = 0.0
    noise_child[idx] = 0.0
    return JointConstraint(Revolute(parent, child, rotation_axis; parent_vertex=parent_vertex+noise_parent, child_vertex=child_vertex+noise_child), name=Symbol(Symbol("$(name_ext):$(kind):c$(j):l$(2*i-1)-$(2*i)")
    ))
end

function create_spherical_joint_constraint(parent, child, parent_vertex, child_vertex, name_ext, i, j, kind; noise=0.0) #2.54e-5)
    # Joint naming convention: joint, cell, link, abbreviated to j, c, and l respectively
    parent_id = parent.id
    child_id = child.id
    noise_parent = randn(3) * noise
    noise_child = randn(3) * noise
    return JointConstraint(Spherical(parent, child; parent_vertex=parent_vertex+noise_parent, child_vertex=child_vertex+noise_child), name=Symbol("$(name_ext):$(kind):cell$(j):l$(parent_id)-$(child_id)"))
end

function create_unit_joint(bodies, i, j, parent_idx_offset, child_idx_offset, name_ext, kind, parent_vertex=nothing, child_vertex=nothing; noise=0.0)
    parent = bodies[2*(i-1) + parent_idx_offset]
    child = bodies[2*i + child_idx_offset]

    if parent_vertex === nothing
        parent_vertex = [0, 0, -parent.shape.rh[2]/2]
    end
    if child_vertex === nothing
        child_vertex = [0, 0, child.shape.rh[2]/2]
    end

    return create_revolute_joint_constraint(parent, child, ROTATION_AXIS, parent_vertex, child_vertex, name_ext, i, j, kind, noise=noise)
end

function create_cell_joint(all_bodies, bodies, i, j, parent_offset, child_idx, name_ext, kind, parent_vertex=nothing, child_vertex=nothing; noise=0.0)
    parent = all_bodies[j-1][end + parent_offset]
    child = bodies[child_idx]
    if parent_vertex === nothing
        parent_vertex = [0, 0, -parent.shape.rh[2]/2]
    end
    if child_vertex === nothing
        child_vertex = [0, 0, child.shape.rh[2]/2]
    end

    return create_revolute_joint_constraint(parent, child, ROTATION_AXIS, parent_vertex, child_vertex, name_ext, i, j, kind, noise=noise)
end

function get_scissor(;num_cells, number_of_units, unit_thickness, unit_length, unit_mass, name_ext, color, parent_vertex, child_vertex, rotation_axis, middle_vertex, noise, kwargs...)
    all_bodies, all_joints = [], []

    for j in 1:num_cells
        bodies = Body{Float64}[]
        joints = JointConstraint{Float64}[]

        for i in 1:number_of_units
            # Creating two bodies and adding them to the 'bodies' list
            for index in [2*i-1, 2*i]
                body_name = Symbol("$(name_ext):c$(j):l$(index)")
                new_body = Cylinder(unit_thickness, unit_length, unit_mass, name=body_name, color=color)
                push!(bodies, new_body)
            end

            # Assuming the last two added bodies are the ones we want to join
            parent_body = bodies[end-1]
            child_body = bodies[end]

            # Create a revolute joint and add it to the 'joints' list
            new_joint = create_revolute_joint_constraint(parent_body, child_body, rotation_axis, middle_vertex, middle_vertex, name_ext, i, j, "x", noise=noise)
            push!(joints, new_joint)

            # If more than 1 unit, connect top and bottom of unit
            if i > 1
                new_joint = create_unit_joint(bodies, i, j, -1, 0, name_ext, "unit", parent_vertex, child_vertex, noise=noise)
                push!(joints, new_joint)
                new_joint = create_unit_joint(bodies, i, j, 0, -1, name_ext, "unit", parent_vertex, child_vertex, noise=noise)
                push!(joints, new_joint)
            end

            # If more than 1 iteration, connect the end bodies from the previous iteration to the current iteration's bodies
            if j > 1 && i == 1
                new_joint = create_cell_joint(all_bodies, bodies, i, j, 0, 1, name_ext, "cell", parent_vertex, child_vertex, noise=noise)
                push!(joints, new_joint)
                new_joint = create_cell_joint(all_bodies, bodies, i, j, -1, 2,name_ext, "cell", parent_vertex, child_vertex, noise=noise)
                push!(joints, new_joint)
            end

            if num_cells==1 && name_ext == "long"
                for index in 1:2
                    body_name = Symbol("$(name_ext):c$num_cells:l$(index):terminal")
                    new_body = Cylinder(unit_thickness, L2, unit_mass, name=body_name, color=color)
                    push!(bodies, new_body)
                end
            end

        end


        push!(all_bodies, bodies)
        push!(all_joints, joints)
    end

    return all_bodies, all_joints
end


function make_connection_joints(num_cells, left_bodies, right_bodies, long_bodies; noise=0.0)
    connection_joints = JointConstraint{Float64}[]

    for i in 1:num_cells

        # Connect short to long (top)
        # if i ==1
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][1], long_bodies[i][1], [0, 0, SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "left_to_long", "", i, "top", noise=noise))

        push!(connection_joints, create_spherical_joint_constraint(right_bodies[i][2], long_bodies[i][2], [0, 0, SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "right_to_long", "", i, "top", noise=noise))
        # end

        # if i < num_cells
        #     # Connect short to long (bottom)
        #     push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][4], long_bodies[i+1][1], [0, 0, -SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "left_to_long", "", i, "bottom"))
        #     push!(connection_joints, create_spherical_joint_constraint(right_bodies[i][3], long_bodies[i+1][2], [0, 0, -SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "right_to_long", "", i, "bottom"))
        # end

        # Connect left to right
        # if i ==1
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][2], right_bodies[i][1], [0, 0, SHORT_LENGTH/2], [0, 0, SHORT_LENGTH/2], "left_to_right", "", i, "top", noise=noise))
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][3], right_bodies[i][4], [0, 0, -SHORT_LENGTH/2], [0, 0, -SHORT_LENGTH/2], "left_to_right", "", i, "bottom", noise=noise))
        # end
    end

    # attach to terminal members
    if num_cells==1
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[num_cells][4], long_bodies[num_cells][3], [0, 0, -SHORT_LENGTH/2], [0, 0, L2/2], "left_to_term", "", num_cells, "bottom", noise=noise))
        push!(connection_joints, create_spherical_joint_constraint(right_bodies[num_cells][3], long_bodies[num_cells][4], [0, 0, -SHORT_LENGTH/2], [0, 0, L2/2], "right_to_term", "", num_cells, "bottom", noise=noise))
    end


    # push!(connection_joints, JointConstraint(Fixed(long_bodies[1][1], long_bodies[1][4]; parent_vertex=PARENT_VERTEX_LONG, child_vertex=[0, 0, -L2/2]), name = :terminal_fixed_joint1))
    # push!(connection_joints, JointConstraint(Fixed(long_bodies[1][2], long_bodies[1][3]; parent_vertex=PARENT_VERTEX_LONG, child_vertex=[0, 0, -L2/2]), name = :terminal_fixed_joint2))

    if num_cells == 1
        push!(connection_joints, create_revolute_joint_constraint(long_bodies[num_cells][1], long_bodies[num_cells][4], ROTATION_AXIS, PARENT_VERTEX_LONG, [0, 0, -L2/2], "terminal", 1, 1, "bottom", noise=noise))
        push!(connection_joints, create_revolute_joint_constraint(long_bodies[num_cells][2], long_bodies[num_cells][3], ROTATION_AXIS, PARENT_VERTEX_LONG, [0, 0, -L2/2], "terminal", 1, 1, "bottom", noise=noise))
    end


    return connection_joints
end

function get_PET(α, NUM_CELL=1, noise=0.0)
    # Scissor Components Setup
    scissor_args = Dict(
        :num_cells => NUM_CELL,
        :unit_thickness => RADIUS,
        :rotation_axis => ROTATION_AXIS,
        :unit_mass => MASS
    )

    # Create Left Bodies
    left_bodies, left_joints = get_scissor(; scissor_args..., unit_length=SHORT_LENGTH, number_of_units=2, name_ext="left", color=RGBA(0.6, 0.6, 0.6, 1.0), middle_vertex=MIDDLE_VERTEX_SHORT, parent_vertex=PARENT_VERTEX_SHORT, child_vertex=CHILD_VERTEX_SHORT, noise=noise);

    # Create Right Bodies
    right_bodies, right_joints = get_scissor(;scissor_args..., unit_length=SHORT_LENGTH, number_of_units=2, name_ext="right", color=RGBA(0.6, 0.6, 0.6, 1.0),middle_vertex=MIDDLE_VERTEX_SHORT, parent_vertex=PARENT_VERTEX_SHORT, child_vertex=CHILD_VERTEX_SHORT, noise=noise);

    # Create Long Bodies
    long_bodies, long_joints = get_scissor(; scissor_args..., unit_length=LONG_LENGTH, number_of_units=1, name_ext="long", color=RGBA(0.6, 0.6, 0.6, 1.0), middle_vertex=MIDDLE_VERTEX_LONG, parent_vertex=PARENT_VERTEX_LONG, child_vertex=CHILD_VERTEX_LONG, noise=noise);

    # Connect Left, Right and Long Bodies via Spherical Joints
    connection_joints = make_connection_joints(NUM_CELL, left_bodies, right_bodies, long_bodies, noise=noise)

    # Mechanism Setup
    origin = Origin()
    all_bodies = vcat(left_bodies..., right_bodies..., long_bodies...)
    joint = JointConstraint(Revolute(origin, long_bodies[1][1], [0, 0, 1]; child_vertex=MIDDLE_VERTEX_LONG, orientation_offset=Dojo.RotX(-pi/2)*Dojo.RotY(-α/2)), name=:origin_joint)
    all_joints = vcat(left_joints..., right_joints..., long_joints..., connection_joints, joint)
    
    return Mechanism(origin, all_bodies, all_joints, gravity=GRAVITY, timestep=TIMESTEP)
end

# Helper function to set configurations
function set_configurations!(mechanism, name, x, q)
    body = get_body(mechanism, name)
    set_maximal_configurations!(body, x=x, q=q)
end

function initialize_PET!(mechanism, α, β, θ, c, NUM_CELL)
    # Rotation constants
    ROT_X_HALF = Dojo.RotX(-pi/2)
    ROT_Y_SHORT = Dojo.RotY(β/2)
    ROT_Y_MINUS_SHORT = Dojo.RotY(-β/2)
    ROT_Y_LONG = Dojo.RotY(α/2)
    ROT_Y_MINUS_LONG = Dojo.RotY(-α/2)
    ROT_Y_LEFT = Dojo.RotY(pi-(pi-θ)/2)
    ROT_Y_RIGHT = Dojo.RotY(pi+(pi-θ)/2)

    # Iterate through cells
    for i in 1:NUM_CELL    
        # Long links
        for j in 1:2
            name = Symbol("long:c$i:l$j")
            vect = [0, 0, L2/2]
            q = ROT_X_HALF * (iseven(j) ? ROT_Y_LONG : ROT_Y_MINUS_LONG)

            x = Dojo.vector_rotate(vect, q) - [0, (c)*(i-1), 0.0]

            set_configurations!(mechanism, name, x, q)
        end

        # Left links
        shift = zeros(3)
        for j in [1]
            name = Symbol("left:c$i:l$j")

            q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

            x_offset = j <= 2 ? 0.0 : 2*(-cos(β/2)*L1)

            x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, L3+L2], ROT_Y_MINUS_LONG) - Dojo.vector_rotate([0, 0, L1], ROT_Y_SHORT) + [0, 0, x_offset], ROT_X_HALF) 

            # x = Dojo.vector_rotate([-sin(α[1]/2)*value(L3+L2)- sin(β[1]/2)*L1, 0, cos(α[1]/2)*value(L3+L2)-cos(β[1]/2)*L1+x_offset],ROT_X_HALF)

            shift = x+Dojo.vector_rotate(CHILD_VERTEX_SHORT, q)
            x = Dojo.vector_rotate((x-shift), ROT_Y_LEFT)
            x += shift
            q = ROT_Y_LEFT * q

            x -= [0, (c)*(i-1), 0.0]

            set_configurations!(mechanism, name, x, q)
        end

        for j in 2:4
            name = Symbol("left:c$i:l$j")
            x_offset = j <= 2 ? 0.0 : 2*(-cos(β/2)*L1)

            x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, L3+L2], ROT_Y_MINUS_LONG) - Dojo.vector_rotate([0, 0, L1], ROT_Y_SHORT) + [0, 0, x_offset], ROT_X_HALF)

            q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

            x = Dojo.vector_rotate((x-shift), ROT_Y_LEFT)
            x += shift
            q = ROT_Y_LEFT * q

            x -= [0, (c)*(i-1), 0.0]

            set_configurations!(mechanism, name, x, q)
        end

        # Right links

        for j in [2]
            name = Symbol("right:c$i:l$j")
            x_offset = j <= 2 ? 0.0 : 2*(-cos(β/2)*L1)
            x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, L3+L2], ROT_Y_LONG) - Dojo.vector_rotate([0, 0, L1], ROT_Y_MINUS_SHORT) + [0, 0, x_offset], ROT_X_HALF)
            q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

            shift = x+Dojo.vector_rotate(CHILD_VERTEX_SHORT, q)
            x = Dojo.vector_rotate((x-shift), ROT_Y_RIGHT)
            x += shift
            q = ROT_Y_RIGHT * q

            x -= [0, (c)*(i-1), 0.0]

            set_configurations!(mechanism, name, x, q)
        end 
        for j in [1,3,4]
            name = Symbol("right:c$i:l$j")
            x_offset = j <= 2 ? 0.0 : 2*(-cos(β/2)*L1)
            x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, L3+L2], ROT_Y_LONG) - Dojo.vector_rotate([0, 0, L1], ROT_Y_MINUS_SHORT) + [0, 0, x_offset], ROT_X_HALF)
            q = ROT_X_HALF  * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

            x = Dojo.vector_rotate((x-shift), ROT_Y_RIGHT)
            x += shift
            q = ROT_Y_RIGHT * q

            x -= [0, (c)*(i-1), 0.0]
            set_configurations!(mechanism, name, x, q)
        end
    end

    # for j in 1:2
    #     # r = get_body(mechanism, Symbol("left:c$NUM_CELL:l$j")).state.r
    #     name = Symbol("long:c$NUM_CELL:l$j:terminal")
    #     # println(name)
    #     # println(mechanism)

    #     vect = [0, 0, -L3+L2/2]
    #     q = ROT_X_HALF * (iseven(j+1) ? ROT_Y_LONG : ROT_Y_MINUS_LONG) 

    #     x = Dojo.vector_rotate(vect, q) 
    #     x -= [0, (c)*(NUM_CELL-1), 0.0]

    #     q = q * (iseven(j) ? ROT_Y_LONG : ROT_Y_MINUS_LONG) * (iseven(j) ? ROT_Y_LONG : ROT_Y_MINUS_LONG)
    #     # println(x, q)

    #     set_configurations!(mechanism, name, x, q)
    # end
end

function run_tolerance_analysis(num_runs, num_poses, num_cells, noise)
    # create vector of vector for errors of each 100 trial and 10 poses
    errors = zeros(num_runs, num_poses) 

    NUM_CELL = num_cells
    # noise = 0.0
    for seed in 1:num_runs
        # Display the run number
        println("Run $seed out of $num_runs...")

        # set random seed 
        srand(seed)
        for ind in 1:num_poses
            # Make Mechanism with noise
            mechanism = get_PET(αs[ind], NUM_CELL, noise)

            # Initialize configuration Guess
            initialize_PET!(mechanism, αs[ind], β_check[ind], θs[ind], cs[ind], NUM_CELL)

            # Newtons method to solve constraints
            res = initialize_constraints!(mechanism, 
                                        fixedids=[get_body(mechanism, Symbol("long:c1:l1")).id, get_body(mechanism, Symbol("long:c1:l2")).id],
                                        regularization=1e-6, #1e-10^(10*i),
                                        lineIter=LINE_ITER, 
                                        newtonIter=30,
                                        debug=DEBUG,
                                        ε = EPSILON)

            # Save Error reading
            errors[seed, ind] = Dojo.loss(mechanism)
                                            
        end
    end
    # Save errors
    save(uniquify("PET_jamming_test_noise_$(noise)_run_$(num_runs)_pose_$(num_poses)_cell_$(num_cells).jld2"), "errors", errors)
    return errors
end
num_cells = 1
num_runs = 50 
noise = 1e-3
num_poses = 10
errors = []
for noise in [2e-4, 5e-4, 2e-3, 5e-3, 1e-2, 2e-2]
    run_tolerance_analysis(num_runs, num_poses, 1, noise)
end

true_data_color = :blue
mean_color = :red
using Plots
using PGFPlotsX  # Import the PGFPlotsX package

Plots.PGFPlotsXBackend()  # Set the PGFPlotsX backend
noises = [1e-6, 1e-5, 1e-4, 2e-4, 5e-4, 1e-3] #2e-3, 5e-3] #, 1e-2, 2e-2]
errors = [load("PET_jamming_test_noise_$(noise)_run_$(num_runs)_pose_$(num_poses)_cell_$(num_cells).jld2")["errors"] for noise in noises]
p = plot(xlabel="θ [rad]", ylabel="Potential [1/m²]", 
xtickfont = font(12),
ytickfont = font(12),guidefontsize = 14,
legendfontsize = 12,
titlefontsize = 16)
for (error, noise) in zip(errors, noises)
    # # Start the plot with the axis labels
    # plot(θs, errors[1,:], alpha=0.3, color=true_data_color, label=nothing, 
    #     xlabel="θ [rad]", ylabel="Potential Energy [N/m²]")

    # # Continue plotting the true data
    # for i in 2:9
    #     plot!(θs, errors[i,:], alpha=0.3, color=true_data_color, label=nothing)
    # end
    # plot!(θs, errors[10,:], alpha=0.3, color=true_data_color, label=nothing)

    # Compute the mean and standard deviation
    mean_values = Dojo.mean(error, dims=1)'
    std_values = Dojo.std(error, dims=1)'

    # Plot the shaded region between the standard deviation
    plot!(p, θs, mean_values, ribbon=std_values, linewidth=3, fillalpha=0.2, label=" Noise=$noise", legend=:topleft)
end
ymin, ymax = ylims()  # Get current y-axis limits
annotate!(p, [(θs[end-1], ymin - 0.1 * (ymax - ymin), text("Deployed", :right, 15)),
          (θs[4], ymin - 0.1 * (ymax - ymin), text("Collapsed", :left, 15))])
savefig(p, "my_plot.tex")
plot!()



# Setting the options for the axis
axis_opts = @pgf {
    xmajorgrids,
    ymajorgrids,
    xlabel = "θ [rad]",
    ylabel = "Potential [1/m²]",
    legend_pos = "north west",
    legend_cell_align="left"
}

# Create the main axis structure
axis_content = []

for (error, noise) in zip(errors, noises)
    # Compute the mean and standard deviation
    mean_values = Dojo.mean(error)
    std_values = Dojo.std(error)

    # Create the plots
    plot_opts = @pgf {
        no_marks,
        "very thick",
        color = "black",  # Choose your desired color
        fill_opacity = 0.2,
        legend_entry = "Noise = $noise"
    }
    push!(axis_content, 
        PlotInc(plot_opts, 
            Table([θs, mean_values .+ std_values, mean_values .- std_values]))
    )
end

# Combining everything into a single axis
scaling_plot = @pgf PGFPlotsX.Axis(axis_opts, axis_content...)


# plot!()
# Annotate the "1 cell" and "10 cell" lines with arrows and text
middle_idx = length(θs) ÷ 2  # Choose a midpoint for the annotation
annotate!([(θs[middle_idx], errors[1, middle_idx], text("1 cell", :left, 10)),
          (θs[middle_idx], errors[10, middle_idx], text("10 cells", :left, 10))])

# Annotate "Deployed" on the left and "Collapsed" on the right of the x-axis
ymin, ymax = ylims()  # Get current y-axis limits
annotate!([(θs[end-1], ymin - 0.1 * (ymax - ymin), text("Deployed", :right, 10)),
          (θs[4], ymin - 0.1 * (ymax - ymin), text("Collapsed", :left, 10))])
plot!()


out = load("PET_jamming_test_noise_0.0_run_2_pose_10_cell_1.jld2")["errors"]
using Plots
plot(θs, out[1,:], label="Run 1")
plot!(θs, out[2,:], label="Run 2")

#########################
errors = load("PET_jamming_test_noise_2e-5_100_10.jld2", "errors")
using Plots
histogram(sum(errors, dims=2)./10, bins=30, alpha=0.7, label="Average Joint Errors")
histogram(sum(zeros(100,10), dims=2)./10, bins=30, alph=0.7, label="Sum of Joint Errors (No Noise)")

plot()
using JLD2
save("PET_jamming_test_noise_2e-5_01.jld2", "errors", errors)

vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)

#######################
# Initialize mechanism
ind = 10
NUM_CELL = 3
noise = 2e-4
mechanism = get_PET(αs[ind], NUM_CELL, noise)
# Getting the degrees of freedom for each joint
joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]

# Creating index ranges for the joints
joint_idx = Dojo.index_ranges(joint_dof)

# Calculating total number of constraints and bodies
num_constraints = sum(joint_dof)
num_bodies = 7*length(mechanism.bodies)


Dojo.loss(mechanism)
# viol = [Dojo.joint_residual_violation(mechanism, joint) for joint in mechanism.joints]
# loss = viol'*I*viol
initialize_PET!(mechanism, αs[ind], β_check[ind], θs[ind], cs[ind], NUM_CELL)
# viol = [Dojo.joint_residual_violation(mechanism, joint) for joint in mechanism.joints]
# loss = viol'*I*viol
Dojo.loss(mechanism)

vis = Visualizer()
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)

# get configuration 
state = get_maximal_state(mechanism)
input = vcat([[state[1+(i-1)*13:3+(i-1)*13]; state[7+(i-1)*13:10+(i-1)*13]] for i in 1:length(mechanism.bodies)]...)
# qs = [ for i in 1:length(mechanism.bodies)]

# set configuration
function set_configurations!(mechanism, input)
    for (i, body) in enumerate(mechanism.bodies)
        x = input[1+(i-1)*7:3+(i-1)*7]
        q = input[4+(i-1)*7:7+(i-1)*7]
        set_maximal_configurations!(body, x=x, q=Quaternion(q...))
    end
end
for (i, body) in enumerate(mechanism.bodies)
    x = input[1+(i-1)*7:3+(i-1)*7]
    q = input[4+(i-1)*7:7+(i-1)*7]
    set_maximal_configurations!(body, x=x, q=Quaternion(q[1], q[2], q[3], q[4]))
end

function cost(mechanism, input)
    for (i, body) in enumerate(mechanism.bodies)
        x = input[1+(i-1)*7:3+(i-1)*7]
        q = input[4+(i-1)*7:7+(i-1)*7]
        set_maximal_configurations!(body, x=x, q=Quaternion(q[1], q[2], q[3], q[4]))
    end

    res = Dojo.Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
    return 0.5*res'*res
end

cost(mechanism, input)

res = initialize_constraints!(mechanism, 
                            fixedids=[],
                            regularization=0.0,
                            lineIter=LINE_ITER, 
                            newtonIter=30,
                            debug=true,
                            ε = EPSILON)

[]
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)


using FiniteDiff
hes = FiniteDiff.finite_difference_hessian(x -> cost(mechanism, x), input)

function get_hes(mechanism)
    # Getting the degrees of freedom for each joint
    joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]

    # Creating index ranges for the joints
    joint_idx = Dojo.index_ranges(joint_dof)

    # Calculating total number of constraints and bodies
    num_constraints = sum(joint_dof)
    num_bodies = 7*length(mechanism.bodies)
    # Initializing constraint Jacobian
    con_jac = Dojo.spzeros(num_constraints, num_bodies)

    # Calculating the constraint results for all joints
    res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
    body_idx = Dojo.index_ranges([7 for _ in mechanism.bodies])
    # Filling the constraint Jacobian
    for (i,(joint, j_idx)) in enumerate(zip(mechanism.joints, joint_idx))
        for (j,(body, b_idx)) in enumerate(zip(mechanism.bodies, body_idx))
            if body.id == joint.parent_id
                con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body)
            elseif body.id == joint.child_id
                con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body)
            end
        end
    end

    hes_dojo = con_jac'*con_jac
end

using FiniteDiff
function newton_method(mechanism, input; tol=1e-6, max_iter=10, line_iter=10)
    # Initialize
    prev_input = 1.0*input
    iter = 0
    norm0 = cost(mechanism, input)
    norm1 = norm0

    while iter < max_iter
        println(iter)
        # Compute gradient and Hessian
        grad = FiniteDiff.finite_difference_gradient(x -> cost(mechanism, x), prev_input)
        hes = FiniteDiff.finite_difference_hessian(x -> cost(mechanism, x), prev_input) + 1e-1*I

        # Newton update
        # Q, R = qr(hes)
        # dx = -R \ (Q' * grad)
        dx = -hes \ grad

        for j = Base.OneTo(line_iter)

            for (i, body) in enumerate(mechanism.bodies)
                # Update x
                input[1+(i-1)*7:3+(i-1)*7] = prev_input[1+(i-1)*7:3+(i-1)*7] + dx[1+(i-1)*7:3+(i-1)*7]*(0.5)^(j-1)

                # Update orientation using a quaternion
                q = Quaternion(prev_input[4+(i-1)*7:7+(i-1)*7]...)
                q̇ = dx[4+(i-1)*7:7+(i-1)*7]
                Δstemp = Dojo.VLᵀmat(q) * q̇
                # Limit quaternion step to feasible length
                if norm(Δstemp) > 1
                    Δstemp = Δstemp/norm(Δstemp)
                end
                w = sqrt(1-min(1, norm((Δstemp*(0.5)^(j-1))))^2)
                input[4+(i-1)*7:7+(i-1)*7] = Dojo.vector(Quaternion(prev_input[4+(i-1)*7:7+(i-1)*7]...) * Quaternion(w, Δstemp*(0.5)^(j-1)...))
                # set_maximal_configurations!(body, x=x, q=Quaternion(q[1], q[2], q[3], q[4]))
            end

            # Compute the maximum constraint violation
            norm1 = cost(mechanism, input) #maximum(violations(mechanism))

            # If violation decreased, exit line search
            if norm1 < norm0 
                # if debug
                    println("exit line search, violation: "*string(norm1))
                # end
                break
            else
                println("new norm: $norm1 vs. old norm: $norm0") 
            end

        end

       
        # Check convergence
        if cost(mechanism, input) < tol
            println("Converged in $iter iterations!")
            return input
        end
        prev_input = 1.0*input
        norm0 = norm1

        iter += 1
    end

    println("Max iterations reached!")
    return input
end
optimized_input = newton_method(mechanism, input)
cost(mechanism, optimized_input)
norm(FiniteDiff.finite_difference_gradient(x -> cost(mechanism, x), optimized_input))

vis = Visualizer()
delete!(vis)
set_configurations!(mechanism, input)
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)

# function cost(mechanism, input)
#     for (i, body) in enumerate(mechanism.bodies)
#         x = input[1+(i-1)*7:3+(i-1)*7]
#         q = input[4+(i-1)*7:7+(i-1)*7]
#         set_maximal_configurations!(body, x=x, q=Quaternion(q[1], q[2], q[3], q[4]))
#     end

#     res = Dojo.Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
#     return res' * I * res
# end

# Example call



qDojo.loss(mechanism)







visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)
# using AppleAccelerate
res = initialize_constraints!(mechanism, 
                                    fixedids=[get_body(mechanism, Symbol("long:c1:l1")).id, get_body(mechanism, Symbol("long:c1:l2")).id],
                                    regularization=1e-5,
                                    lineIter=LINE_ITER, 
                                    newtonIter=NEWTON_ITER,
                                    debug=DEBUG,
                                    ε = EPSILON)
Dojo.loss(mechanism)
for i in 0:10
    # println("Initializing constraints: iteration $i out of 10")
    try
        res = initialize_constraints!(mechanism, 
                                    fixedids=[get_body(mechanism, Symbol("long:c1:l1")).id, get_body(mechanism, Symbol("long:c1:l2")).id],
                                    regularization=1e-10^(10*i),
                                    lineIter=LINE_ITER, 
                                    newtonIter=NEWTON_ITER,
                                    debug=DEBUG,
                                    ε = EPSILON)
                                
        if i == 10
            # push!(errors, res)
            # errors[seed, ind] = res
            if res === nothing
                errors_over_cells[seed, ind] = 0.0
            else
                errors_over_cells[seed, ind] = res
            end
            # errors_no_noise[seed, ind] = res
        end
        # vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)
    catch e
        println("Error during initialization on iteration $i: $e")
    end
end
Dojo.loss(mechanism)
vis = Visualizer()
delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.005)
# viol = [Dojo.joint_residual_violation(mechanism, joint) for joint in mechanism.joints]
# loss = viol'*I*viol

# Dojo.violation(mchanism)

# create vector of vector for errors of each 100 trial and 10 poses
errors = zeros(10, 10) 
errors_no_noise = zeros(1, 10)
errors_over_cells = zeros(10, 10)
NUM_CELL = 3
noise = 0.0
for seed in 1:10
    # set random seed 
    NUM_CELL = seed
    srand(seed)
    for ind in 1:10
        # a = as[ind]
        # b = bs[ind]
        c = cs[ind]
        # d1 = d1s[ind]
        # d2 = d2s[ind]
        α = αs[ind]
        β = β_check[ind]
        θ = θs[ind] #acos(max(-1.0, (a^2-2*b^2)/(-2*b^2))) # cos theta
        # ind = 10
        # NUM_CELL = 3
        mechanism = get_PET(αs[ind], NUM_CELL, noise)
        initialize_PET!(mechanism, αs[ind], β_check[ind], θs[ind], cs[ind], NUM_CELL)
        # for i in 0:10
        #     # println("Initializing constraints: iteration $i out of 10")
        #     try
        res = initialize_constraints!(mechanism, 
                                    fixedids=[get_body(mechanism, Symbol("long:c1:l1")).id, get_body(mechanism, Symbol("long:c1:l2")).id],
                                    regularization=1e-6, #1e-10^(10*i),
                                    lineIter=LINE_ITER, 
                                    newtonIter=NEWTON_ITER,
                                    debug=DEBUG,
                                    ε = EPSILON)
        errors[seed, ind] = Dojo.loss(mechanism)
                                        
        #         if i == 10
        #             # push!(errors, res)
        #             # errors[seed, ind] = res
        #             if res === nothing
        #                 errors_over_cells[seed, ind] = 0.0
        #             else
        #                 errors_over_cells[seed, ind] = res
        #             end
        #             # errors_no_noise[seed, ind] = res
        #         end
        #         # vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)
        #     catch e
        #         println("Error during initialization on iteration $i: $e")
        #     end
        # end
      
    end
end
# errors
save("PET_jamming_test_noise_2e-5_100_10.jld2", "errors", errors)
errors = load("PET_jamming_test_noise_2e-5_100_10.jld2", "errors")
using Plots
histogram(sum(errors, dims=2)./10, bins=30, alpha=0.7, label="Average Joint Errors")
histogram(sum(zeros(100,10), dims=2)./10, bins=30, alph=0.7, label="Sum of Joint Errors (No Noise)")

plot()
using JLD2
save("PET_jamming_test_noise_2e-5_01.jld2", "errors", errors)

vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.005)


# function constraintstep!(mechanism::Mechanism{T}, freebodies::Vector{Body{T}}; regularization=1e-6) where T
# Fetching all the free bodies
# freebodies = [get_body(mechanism, id) for id in freeids]

# Getting the degrees of freedom for each joint
joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]

# Creating index ranges for the joints
joint_idx = Dojo.index_ranges(joint_dof)

# Calculating total number of constraints and bodies
num_constraints = sum(joint_dof)
num_bodies = 6*length(mechanism.bodies)

# Creating index ranges for the bodies
body_idx = Dojo.index_ranges([7 for _ in mechanism.bodies])

# Initializing constraint Jacobian
con_jac = Dojo.spzeros(num_constraints, num_bodies)

# Calculating the constraint results for all joints
res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))

# Filling the constraint Jacobian
for (i,(joint, j_idx)) in enumerate(zip(mechanism.joints, joint_idx))
    for (j,(body, b_idx)) in enumerate(zip(mechanism.bodies, body_idx))
        if body.id == joint.parent_id
            con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body)
        elseif body.id == joint.child_id
            con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body)
        end
    end
end

function get_joint_state(mechanism, joint)
    ## Current state
    x = []
    r = zeros(Float64,0)
    v = zeros(Float64,0)
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    for element in (joint.translational, joint.rotational)
        pos = minimal_coordinates(element, pbody, cbody)
        vel = minimal_velocities(element, pbody, cbody, mechanism.timestep)
			push!(r, pos...)
			push!(v, vel...)
    end
    push!(x, [r; v]...)
    return x
end

joint = get_joint(mechanism, Symbol("long:x:c1:l1-2"))
x_initial = get_joint_state(mechanism, joint)
states = Vector{Vector{Float64}}()
push!(states, x_initial)
function controller!(mechanism, k)
    # K = 0.001
    # D = 0.0
    ## Target state
    # x_goal = [αs[end], 0.0]

    

    ## Control inputs
    # u = -Dojo.dot([K, D], (x - x_goal))
    # println(u)
    ## 3 ways to set input:

    ## 1: get joint and set input
    set_input!(joint, [0.001])
    state = get_joint_state(mechanism, joint)
    push!(states, state)
    # cart = get_body(mechanism, :cart)
    # set_external_force!(cart; torque=[0;u;0])
end

# storage = simulate!(mechanism, 0.1, controller!, record=true)
opts = SolverOptions(verbose=true, reg=1e-4, max_iter=100)
storage = simulate!(mechanism, 100*mechanism.timestep, controller!, record=true, opts=opts)
vis = visualize(mechanism, storage, vis=vis)
render(vis)

using Plots
times = collect(0:100).*mechanism.timestep
plot!(times, [state[1] for state in states], label="α_true")
plot!(times, [state[2] for state in states], label="αdot_true")
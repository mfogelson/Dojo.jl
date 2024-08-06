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
pathname = "HERDS_design_optimization_final"
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
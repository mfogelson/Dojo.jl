using Dojo
using JuMP
using Ipopt
using Juniper
model = Model(Ipopt.Optimizer)

# Decision variables 
states = 11
n = 2 # Number of small member cells

# @variable(model, 0.01 <= l1 <= 10.0) # Half of one link
# @variable(model, 0.01 <= l2 <= 10.0) # The extension 
# @variable(model, 0.01 <= l3 <= 10.0) # Half of one link
# @variable(model,  0.0 <= α[1:states] <= pi) # For the big members

@variable(model, 0.0 <= β[1:states] <= pi) # for the small Members 

# Expressions
a = @NLexpression(model, [i=1:states], (l2+l3)*sqrt(2*(1-cos(α[i])))) # Width 
b = @NLexpression(model, [i=1:states], l1*sqrt(2*(1-cos(β[i])))) # Edge
c = @NLexpression(model, [i=1:states], n*l1*sqrt(2*(1-cos(pi-β[i])))) # Extension
d1 = @NLexpression(model, [i=1:states], sqrt((l2+l3)^2+l3^2-2*(l3*(l2+l3))*cos(pi-α[i])))
d2 = @NLexpression(model, [i=1:states], sqrt((l3)^2+l2^2-2*(l3*(l2))*cos(α[i])))
θ = @NLexpression(model, [i=1:states], (a[i]^2-2*b[i]^2)/(-2*b[i]^2)) # cos theta

# Constraints
@NLconstraint(model, [i=1:states], a[i] <= 2*b[i]) # Pop triangle equation
@NLconstraint(model, [i=1:states],c[i] <= l2 + d1[i]) # Extension triangle constriants
@NLconstraint(model,[i=1:states],c[i] + l2 >= d1[i]) # Extension triangle constriants
@NLconstraint(model, [i=1:states], d1[i]^2+d2[i]^2-c[i]^2-l3^2-2*(l2*(l2+l3))==0) # Trapizoid parallel constriants
@NLconstraint(model,[i=1:states], a[i] >= 0.01)
@NLconstraint(model, [i=1:states],b[i] >= 0.01)
@NLconstraint(model,[i=1:states], c[i] >= 0.01)
@constraint(model, [i = 1:states-1], α[i+1] <= α[i])
@constraint(model, [i = 1:states-1], β[i+1] <= β[i])

# Design specific constraints
# @NLconstraint(model, (2*l3+l2)*cos(α[10]/2)-1.0 == 0)
@NLconstraint(model, c[states] - 2.0 == 0.0)

# Objective function
@NLobjective(model, Min, (θ[1]-cos(pi))^2 + (θ[10]-cos(pi/3))^2 + (c[1]/c[10]) + (cos(α[10])-cos(pi/3))^2 + (cos(β[10])-cos(pi/3))^2)

optimize!(model)

l1 = value(l1)
l2 = value(l2)
l3 = value(l3)
α = value.(α)
β = value.(β)
θ = acos.(value.(θ))
a = value.(a)
b = value.(b)
c = value.(c)
d1 = value.(d1)
d2 = value.(d2)

println("l1 = ", l1)
println("l2 = ", l2)
println("l3 = ", l3)
println("α = ", α)
println("β = ", β)
println("θ = ", θ)
println("a = ", a)
println("b = ", b)
println("c = ", c)


# ### Parameters
const RADIUS = 0.01
const SHORT_LENGTH = 2*l1
const LONG_LENGTH = l2 + 2*l3
const BOT_LENGTH = 2*l3
const MASS = 0.001

const PARENT_VERTEX_LONG = [0, 0, -LONG_LENGTH/2]
const PARENT_VERTEX_SHORT = [0, 0, -SHORT_LENGTH/2]
const CHILD_VERTEX_LONG = [0, 0, BOT_LENGTH/2]
const CHILD_VERTEX_SHORT = [0, 0, SHORT_LENGTH/2]
const MIDDLE_VERTEX_LONG = [0, 0, -(LONG_LENGTH - BOT_LENGTH)/2]
const MIDDLE_VERTEX_SHORT = [0, 0, 0.0]
const ROTATION_AXIS = [0, 1, 0]

# Sim params
const DAMPER = 0.0001
const TIMESTEP = 0.001
const REG = 1e-6
const GRAVITY = [0, 0, 0.0]

# System Params
const NUM_CELLS = 1

# ### Helper Functions
function create_revolute_joint_constraint(parent, child, rotation_axis, parent_vertex, child_vertex, name_ext, i, j, kind)
    # Joint naming convention: joint, cell, link, abbreviated to j, c, and l respectively

    return JointConstraint(Revolute(parent, child, rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol(Symbol("$(name_ext):$(kind):c$(j):l$(2*i-1)-$(2*i)")
    ))
end

function create_spherical_joint_constraint(parent, child, parent_vertex, child_vertex, name_ext, i, j, kind)
    # Joint naming convention: joint, cell, link, abbreviated to j, c, and l respectively
    parent_id = parent.id
    child_id = child.id

    return JointConstraint(Spherical(parent, child; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext):$(kind):cell$(j):l$(parent_id)-$(child_id)"))
end

function create_unit_joint(bodies, i, j, parent_idx_offset, child_idx_offset, name_ext, kind, parent_vertex=nothing, child_vertex=nothing)
    parent = bodies[2*(i-1) + parent_idx_offset]
    child = bodies[2*i + child_idx_offset]

    if parent_vertex === nothing
        parent_vertex = [0, 0, -parent.shape.rh[2]/2]
    end
    if child_vertex === nothing
        child_vertex = [0, 0, child.shape.rh[2]/2]
    end

    return create_revolute_joint_constraint(parent, child, ROTATION_AXIS, parent_vertex, child_vertex, name_ext, i, j, kind)
end

function create_cell_joint(all_bodies, bodies, i, j, parent_offset, child_idx, name_ext, kind, parent_vertex=nothing, child_vertex=nothing)
    parent = all_bodies[j-1][end + parent_offset]
    child = bodies[child_idx]
    if parent_vertex === nothing
        parent_vertex = [0, 0, -parent.shape.rh[2]/2]
    end
    if child_vertex === nothing
        child_vertex = [0, 0, child.shape.rh[2]/2]
    end

    return create_revolute_joint_constraint(parent, child, ROTATION_AXIS, parent_vertex, child_vertex, name_ext, i, j, kind)
end

function get_scissor(;num_cells, number_of_units, unit_thickness, unit_length, unit_mass, name_ext, color, parent_vertex, child_vertex, rotation_axis, middle_vertex, kwargs...)
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
            new_joint = create_revolute_joint_constraint(parent_body, child_body, rotation_axis, middle_vertex, middle_vertex, name_ext, i, j, "x")
            push!(joints, new_joint)

            # If more than 1 unit, connect top and bottom of unit
            if i > 1
                new_joint = create_unit_joint(bodies, i, j, -1, 0, name_ext, "unit", parent_vertex, child_vertex)
                push!(joints, new_joint)
                new_joint = create_unit_joint(bodies, i, j, 0, -1, name_ext, "unit", parent_vertex, child_vertex)
                push!(joints, new_joint)
            end

            # If more than 1 iteration, connect the end bodies from the previous iteration to the current iteration's bodies
            if j > 1
                new_joint = create_cell_joint(all_bodies, bodies, i, j, 0, 1, name_ext, "cell", parent_vertex, child_vertex)
                push!(joints, new_joint)
                new_joint = create_cell_joint(all_bodies, bodies, i, j, -1, 2,name_ext, "cell", parent_vertex, child_vertex)
                push!(joints, new_joint)
            end
        end
        push!(all_bodies, bodies)
        push!(all_joints, joints)
    end

    return all_bodies, all_joints
end


function make_connection_joints(num_cells, left_bodies, right_bodies, long_bodies)
    connection_joints = JointConstraint{Float64}[]

    for i in 1:num_cells

        # Connect short to long (top)
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][1], long_bodies[i][1], [0, 0, SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "left_to_long", "", i, "top"))

        push!(connection_joints, create_spherical_joint_constraint(right_bodies[i][2], long_bodies[i][2], [0, 0, SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "right_to_long", "", i, "top"))

        if i < num_cells
            # Connect short to long (bottom)
            push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][4], long_bodies[i+1][1], [0, 0, -SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "left_to_long", "", i, "bottom"))
            push!(connection_joints, create_spherical_joint_constraint(right_bodies[i][3], long_bodies[i+1][2], [0, 0, -SHORT_LENGTH/2], [0, 0, LONG_LENGTH/2], "right_to_long", "", i, "bottom"))
        end
    
        # Connect left to right
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][2], right_bodies[i][1], [0, 0, SHORT_LENGTH/2], [0, 0, SHORT_LENGTH/2], "left_to_right", "", i, "top"))
        push!(connection_joints, create_spherical_joint_constraint(left_bodies[i][3], right_bodies[i][4], [0, 0, -SHORT_LENGTH/2], [0, 0, -SHORT_LENGTH/2], "left_to_right", "", i, "bottom"))
    end

    return connection_joints
end

# Scissor Components Setup
scissor_args = Dict(
    :num_cells => NUM_CELLS,
    :unit_thickness => RADIUS,
    :rotation_axis => ROTATION_AXIS,
    :unit_mass => MASS
)

# Create Left Bodies
left_bodies, left_joints = get_scissor(; scissor_args..., unit_length=SHORT_LENGTH, number_of_units=n, name_ext="left", color=RGBA(0.6, 0.6, 0.6, 1.0), middle_vertex=MIDDLE_VERTEX_SHORT, parent_vertex=PARENT_VERTEX_SHORT, child_vertex=CHILD_VERTEX_SHORT);

# Create Right Bodies
right_bodies, right_joints = get_scissor(;scissor_args..., unit_length=SHORT_LENGTH, number_of_units=n, name_ext="right", color=RGBA(0.6, 0.6, 0.6, 1.0),middle_vertex=MIDDLE_VERTEX_SHORT, parent_vertex=PARENT_VERTEX_SHORT, child_vertex=CHILD_VERTEX_SHORT);

# Create Long Bodies
long_bodies, long_joints = get_scissor(; scissor_args..., unit_length=LONG_LENGTH, number_of_units=1, name_ext="long", color=RGBA(0.6, 0.6, 0.6, 1.0), middle_vertex=MIDDLE_VERTEX_LONG, parent_vertex=PARENT_VERTEX_LONG, child_vertex=CHILD_VERTEX_LONG);

# Connect Left, Right and Long Bodies via Spherical Joints
connection_joints = make_connection_joints(NUM_CELLS, left_bodies, right_bodies, long_bodies)

# Mechanism Setup
all_bodies = vcat(left_bodies..., right_bodies..., long_bodies...)
all_joints = vcat(left_joints..., right_joints..., long_joints..., connection_joints)
origin = Origin()
mechanism = Mechanism(origin, all_bodies, all_joints, gravity=GRAVITY, timestep=TIMESTEP)

# Rotation constants
ind = 1
ROT_X_HALF = Dojo.RotX(-pi/2)
ROT_Y_SHORT = Dojo.RotY(β[ind]/2)
ROT_Y_MINUS_SHORT = Dojo.RotY(-β[ind]/2)
ROT_Y_LONG = Dojo.RotY(α[ind]/2)
ROT_Y_MINUS_LONG = Dojo.RotY(-α[ind]/2)
ROT_Y_LEFT = Dojo.RotY(pi-(pi-θ[ind])/2)
ROT_Y_RIGHT = Dojo.RotY(pi+(pi-θ[ind])/2)

# Helper function to set configurations
function set_configurations!(mechanism, name, x, q)
    body = get_body(mechanism, name)
    set_maximal_configurations!(body, x=x, q=q)
end

# Iterate through cells
for i in 1:NUM_CELLS    
    # Long links
    for j in 1:2
        name = Symbol("long:c$i:l$j")
        vect = [0, 0, l2/2]
        q = ROT_X_HALF * (iseven(j) ? ROT_Y_LONG : ROT_Y_MINUS_LONG)

        x = Dojo.vector_rotate(vect, q) + [0, value.(c)[ind]*(i-1), 0.0]

        set_configurations!(mechanism, name, x, q)
    end
    
    # Left links
    shift = zeros(3)
    for j in [1]
        name = Symbol("left:c$i:l$j")

        q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

        x_offset = j <= 2 ? 0.0 : 2*(-cos(β[ind]/2)*l1)

        x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, l3+l2], ROT_Y_MINUS_LONG) - Dojo.vector_rotate([0, 0, l1], ROT_Y_SHORT) + [0, 0, x_offset], ROT_X_HALF) 

        # x = Dojo.vector_rotate([-sin(α[1]/2)*value(l3+l2)- sin(β[1]/2)*l1, 0, cos(α[1]/2)*value(l3+l2)-cos(β[1]/2)*l1+x_offset],ROT_X_HALF)

        shift = x+Dojo.vector_rotate(CHILD_VERTEX_SHORT, q)
        x = Dojo.vector_rotate((x-shift), ROT_Y_LEFT)
        x += shift
        q = ROT_Y_LEFT * q

        x += [0, value.(c)[ind]*(i-1), 0.0]

        set_configurations!(mechanism, name, x, q)
    end

    for j in 2:4
        name = Symbol("left:c$i:l$j")
        x_offset = j <= 2 ? 0.0 : 2*(-cos(β[ind]/2)*l1)

        x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, l3+l2], ROT_Y_MINUS_LONG) - Dojo.vector_rotate([0, 0, l1], ROT_Y_SHORT) + [0, 0, x_offset], ROT_X_HALF)

        q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

        x = Dojo.vector_rotate((x-shift), ROT_Y_LEFT)
        x += shift
        q = ROT_Y_LEFT * q

        x += [0, value.(c)[ind]*(i-1), 0.0]

        set_configurations!(mechanism, name, x, q)
    end
    
    # Right links
    
    for j in [2]
        name = Symbol("right:c$i:l$j")
        x_offset = j <= 2 ? 0.0 : 2*(-cos(β[ind]/2)*l1)
        x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, l3+l2], ROT_Y_LONG) - Dojo.vector_rotate([0, 0, l1], ROT_Y_MINUS_SHORT) + [0, 0, x_offset], ROT_X_HALF)
        q = ROT_X_HALF * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

        shift = x+Dojo.vector_rotate(CHILD_VERTEX_SHORT, q)
        x = Dojo.vector_rotate((x-shift), ROT_Y_RIGHT)
        x += shift
        q = ROT_Y_RIGHT * q

        x += [0, value.(c)[ind]*(i-1), 0.0]

        set_configurations!(mechanism, name, x, q)
    end 
    for j in [1,3,4]
        name = Symbol("right:c$i:l$j")
        x_offset = j <= 2 ? 0.0 : 2*(-cos(β[ind]/2)*l1)
        x = Dojo.vector_rotate(Dojo.vector_rotate([0, 0, l3+l2], ROT_Y_LONG) - Dojo.vector_rotate([0, 0, l1], ROT_Y_MINUS_SHORT) + [0, 0, x_offset], ROT_X_HALF)
        q = ROT_X_HALF  * (iseven(j) ? ROT_Y_MINUS_SHORT : ROT_Y_SHORT)

        x = Dojo.vector_rotate((x-shift), ROT_Y_RIGHT)
        x += shift
        q = ROT_Y_RIGHT * q

        x += [0, value.(c)[ind]*(i-1), 0.0]
        set_configurations!(mechanism, name, x, q)
    end
end

if !isdefined(Main, :vis)
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.05)

const LINE_ITER = 10
const NEWTON_ITER = 400
const DEBUG = false
const EPSILON = 1e-10

for i in 0:10
    println("Initializing constraints: iteration $i out of 10")
    try
        initialize_constraints!(mechanism, 
                                fixedids=[],
                                regularization=1e-10*(10^i),
                                lineIter=LINE_ITER, 
                                newtonIter=NEWTON_ITER,
                                debug=DEBUG,
                                ε = EPSILON)
    catch e
        println("Error during initialization on iteration $i: $e")
    end
end

vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.05)


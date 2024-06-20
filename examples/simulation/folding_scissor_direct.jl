# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Parameters
# Link params
radius = 0.1
short_length = 44.60/10

long_length = 2.0*short_length; #77.68/10
bot_length = 0.5*long_length #68.42/10 #3/2*short_length #44.60/10 #68.42/10
mass = 0.001
parent_vertex = [0, 0, bot_length/2]
middle_vertex = [0, 0, -(long_length-bot_length)/4]
# Sim params
damper = 0.0001
timestep = 0.001
reg = 1e-6
gravity = [0, 0, 0.0]

# System Params
num_cells = 2

# ### Make system
origin = Origin()
# short_members = [Body{Float64}[] for i in 1:num_cells]
# long_members = [Body{Float64}[] for i in 1:num_cells]
# joints = [JointConstraint{Float64}[] for i in 1:num_cells]

function get_scissor(num_cells, number_of_units, unit_thickness, unit_length, unit_mass; name_ext="scissor", rotation_axis=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75), middle_vertex=zeros(3), parent_vertex=[0, 0, -unit_length/2.0], child_vertex=[0, 0, unit_length/2.0])
    all_bodies = []
    all_joints = []
    for j in 1:num_cells
        # number_of_units = floor(Int, beam_length/(unit_width))
        # number_of_links = 2*number_of_units
        # number_of_joints = 3*number_of_units-4

        # ### Scissor components
        bodies = Body{Float64}[]
        joints = JointConstraint{Float64}[]
        
        for i in 1:number_of_units
            body = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i-1)_cell$(j)"), color=color)

            body2 = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i)_cell$(j)"), color=color)

            push!(bodies, body)
            push!(bodies, body2)
        
            
            joint2 = JointConstraint(Revolute(body, body2, rotation_axis, parent_vertex=middle_vertex, child_vertex=middle_vertex), name=Symbol("$(name_ext)_joint_$(2i-1)_to_$(2*i)_cell$(j)"))
            # joint2 = JointConstraint(Spherical(body, body2, parent_vertex=middle_vertex, child_vertex=middle_vertex), name=Symbol("$(name_ext)_joint_$(2i-1)_to_$(2*i)_cell$(j)"))
            push!(joints, joint2)

            if i > 1
                # println(bodies[2*(i-1)-1].shape.rh)
                # println(bodies[2*i])
                joint1 = JointConstraint(Revolute(bodies[2*(i-1)-1], bodies[2*i], rotation_axis; parent_vertex=[0, 0, -bodies[2*(i-1)-1].shape.rh[2]/2], child_vertex=[0, 0, bodies[2i].shape.rh[2]/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1)-1)_to_$(2*i)_cell$(j)"))
                # joint1 = JointConstraint(Spherical(bodies[1], bodies[4]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*(i-1)-1)_to_$(2*i)_cell$(j)"))

                joint2 = JointConstraint(Revolute(bodies[2*(i-1)], bodies[2*i-1], rotation_axis; parent_vertex=[0, 0, -bodies[2*(i-1)].shape.rh[2]/2], child_vertex=[0, 0, bodies[2i-1].shape.rh[2]/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1))_to_$(2*i-1)_cell$(j)"))
                # joint2 = JointConstraint(Spherical(bodies[2], bodies[3]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*(i-1))_to_$(2*i-1)_cell$(j)"))

                push!(joints, joint1)
                push!(joints, joint2)
            end
            
        end

        if j > 1
            joint1 = JointConstraint(Revolute(all_bodies[j-1][end], bodies[1], rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*number_of_units)_to_$(1)_cell$(j-1)_to_cell$(j)"))
            # joint1 = JointConstraint(Spherical(all_bodies[j-1][1], bodies[2]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*(j-1)-1)_to_$(2*j)_cell$(j)"))


            joint2 = JointConstraint(Revolute(all_bodies[j-1][end-1], bodies[2], rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*number_of_units-1)_to_$(2)_cell$(j-1)_to_cell$(j)"))
            # JointConstraint(Spherical(all_bodies[j-1][2], bodies[1], rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("$(name_ext)_joint_$(2*(j-1))_to_$(2*j-1)_cell$(j)"))

            push!(joints, joint1)
            push!(joints, joint2)
        end

        push!(all_bodies, bodies)
        push!(all_joints, joints)
    end

    return all_bodies, all_joints
end

left_bodies, left_joints = get_scissor(num_cells, 2, radius, short_length, mass, name_ext="left", rotation_axis=[0; 1; 0], color=RGBA(1.0, 0, 0))
right_bodies, right_joints = get_scissor(num_cells, 2, radius, short_length, mass, name_ext="right", rotation_axis=[0; 1; 0], color=RGBA(0, 1.0, 0))
long_bodies, long_joints = get_scissor(num_cells, 1, radius, long_length, mass, name_ext="long", rotation_axis=[0; 1; 0], color=RGBA(0, 0, 1.0), middle_vertex=middle_vertex, parent_vertex=[0, 0, -long_length/2], child_vertex=parent_vertex)

connection_joints = JointConstraint{Float64}[]
# ### Make bodies
for i in 1:num_cells

    # short to long top 
    joint = JointConstraint(Spherical(left_bodies[i][1], long_bodies[i][1], parent_vertex=[0, 0, short_length/2], child_vertex=[0, 0, long_length/2]), name=Symbol("left_to_long_top_$(i)"))
    push!(connection_joints, joint)
    joint = JointConstraint(Spherical(right_bodies[i][2], long_bodies[i][2], parent_vertex=[0, 0, short_length/2], child_vertex=[0, 0, long_length/2]), name=Symbol("right_to_long_top_$(i)"))
    push!(connection_joints, joint)

    if num_cells > 1 && i < num_cells
        joint = JointConstraint(Spherical(left_bodies[i][4], long_bodies[i+1][1], parent_vertex=[0, 0, -short_length/2], child_vertex=[0, 0, long_length/2]), name=Symbol("left_to_long_bottom_$(i)"))
        push!(connection_joints, joint)
        joint = JointConstraint(Spherical(right_bodies[i][3], long_bodies[i+1][2], parent_vertex=[0, 0, -short_length/2], child_vertex=[0, 0, long_length/2]), name=Symbol("right_to_long_top_$(i)"))
        push!(connection_joints, joint)
    end

    # Connect left to right
    joint = JointConstraint(Spherical(left_bodies[i][2], right_bodies[i][1], parent_vertex=[0, 0, short_length/2], child_vertex=[0, 0, short_length/2]), name=Symbol("left_2_to_right_1_$(i)"))
    push!(connection_joints, joint)
    # joint = JointConstraint(Spherical(left_bodies[i][1], right_bodies[i][2], parent_vertex=[0, 0, -short_length/2], child_vertex=[0, 0, -short_length/2]), name=Symbol("left_1_to_right_2_$(i)"))
    # push!(connection_joints, joint)

    # joint = JointConstraint(Spherical(left_bodies[i][4], right_bodies[i][3], parent_vertex=[0, 0, short_length/2], child_vertex=[0, 0, short_length/2]), name=Symbol("left_4_to_right_3_$(i)"))
    # push!(connection_joints, joint)
    joint = JointConstraint(Spherical(left_bodies[i][3], right_bodies[i][4], parent_vertex=[0, 0, -short_length/2], child_vertex=[0, 0, -short_length/2]), name=Symbol("left_3_to_right_4_$(i)"))
    push!(connection_joints, joint)


end

# ### Make mechanism 
all_bodies = vcat(left_bodies..., right_bodies..., long_bodies...)
all_joints = vcat(left_joints..., right_joints..., long_joints..., connection_joints)
mechanism = Mechanism(origin, all_bodies, all_joints, gravity=gravity, timestep=timestep)
# mechanism = Mechanism(origin, vcat(long_bodies...), vcat(long_joints...), gravity=gravity, timestep=timestep)
# mechanism = Mechanism(origin, vcat(left_bodies...), vcat(left_joints...), gravity=gravity, timestep=timestep)

# ### Guess initial state
for i in 1:num_cells
    set_maximal_configurations!(get_body(mechanism, Symbol("long_link1_cell$(i)")), x=Dojo.vector_rotate([0, 0, -short_length*(i-1)], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotY(-2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("long_link2_cell$(i)")), x=Dojo.vector_rotate([0, 0, -short_length*(i-1)], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotY(2pi/8))

    set_maximal_configurations!(get_body(mechanism, Symbol("left_link1_cell$(i)")), x=Dojo.vector_rotate([-cos(3pi/8)*long_length/2, -1.0, -short_length*(i-1)+sin(pi/8)*short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(-pi/7)*Dojo.RotY(-2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("left_link2_cell$(i)")), x=Dojo.vector_rotate([-cos(3pi/8)*long_length/2, -1, -short_length*(i-1)+sin(pi/8)*short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(-pi/7)*Dojo.RotY(2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("left_link3_cell$(i)")), x=Dojo.vector_rotate([-cos(3pi/8)*long_length/2, -1, -short_length*(i-1)-short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(-pi/7)*Dojo.RotY(-2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("left_link4_cell$(i)")), x=Dojo.vector_rotate([-cos(3pi/8)*long_length/2, -1, -short_length*(i-1)-short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(-pi/7)*Dojo.RotY(2pi/8))


    set_maximal_configurations!(get_body(mechanism, Symbol("right_link1_cell$(i)")), x=Dojo.vector_rotate([cos(3pi/8)*long_length/2, -1, -short_length*(i-1)+sin(pi/8)*short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(pi/7)*Dojo.RotY(-2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("right_link2_cell$(i)")), x=Dojo.vector_rotate([cos(3pi/8)*long_length/2, -1, -short_length*(i-1)+sin(pi/8)*short_length/2],Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(pi/7)*Dojo.RotY(2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("right_link3_cell$(i)")), x=Dojo.vector_rotate([cos(3pi/8)*long_length/2, -1, -short_length*(i-1)-short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(pi/7)*Dojo.RotY(-2pi/8))
    set_maximal_configurations!(get_body(mechanism, Symbol("right_link4_cell$(i)")), x=Dojo.vector_rotate([cos(3pi/8)*long_length/2, -1, -short_length*(i-1)-short_length/2], Dojo.RotX(-pi/2)), q=Dojo.RotX(-pi/2)*Dojo.RotZ(pi/7)*Dojo.RotY(2pi/8))
end
set_dampers!(mechanism.joints, 0.1)
if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)
for i in 1:10
    initialize_constraints!(mechanism, fixedids=[], regularization=1e-10*(10^i), lineIter=10, newtonIter=300, debug=false, ε = 1e-10)
end
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)



# set_maximal_configurations!(get_body(mechanism, Symbol("long_link1_cell1")), x=get_body(mechanism, Symbol("long_link1_cell1")).state.x2, q=Dojo.RotY(-3pi/8))
# set_maximal_configurations!(get_body(mechanism, Symbol("long_link2_cell1")), x=get_body(mechanism, Symbol("long_link2_cell1")).state.x2, q=Dojo.RotY(3pi/8))


# initialize_constraints!(mechanism, fixedids=[get_body(mechanism, Symbol("long_link1_cell1")).id, get_body(mechanism, Symbol("long_link2_cell1")).id], regularization=0.0, lineIter=10, newtonIter=300, debug=true)
# vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)

zero_velocities!(mechanism)
function control!(mechanism, t)
    node1 = get_body(mechanism, Symbol("long_link1_cell2"))
    node2 = get_body(mechanism, Symbol("long_link2_cell2"))
    angle = minimal_coordinates(mechanism.joints[end-12], node1, node2)
    if angle[1] < deg2rad(45)
        zero_velocities!(mechanism)
        return
    end 
    set_external_force!(get_body(mechanism, Symbol("long_link1_cell1")), force=[0, 0, 0], torque=[0,0.100, 0])
    set_external_force!(get_body(mechanism, Symbol("long_link2_cell1")), force=[0, 0, 0], torque=[0, -0.100, 0])
    set_external_force!(get_body(mechanism, Symbol("long_link1_cell2")), force=[0, 0, 0], torque=[0, 0.100, 0])
    set_external_force!(get_body(mechanism, Symbol("long_link2_cell2")), force=[0, 0, 0], torque=[0, -0.100, 0])
end
    
mechanism.gravity = [0, 0, 0.0]
opts = SolverOptions(verbose=false, reg=1e-6, max_iter=100)
storage = simulate!(mechanism, 10000*timestep, control!, record=true, opts=opts)
delete!(vis)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.05)


compressed_state = get_maximal_state(mechanism)
using JLD2
save("compressed_state_folding_scissor_2_cell.jld2", "compressed_state", compressed_state)

expanding_simulation = storage
save("expanding_simulation_folding_scissor_2_cell.jld2", "expanding_simulation", expanding_simulation)

save("mechanism_folding_scissor_2_cell.jld2", "mechanism", mechanism)
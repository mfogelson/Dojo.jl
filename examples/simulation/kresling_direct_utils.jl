using Dojo

function get_kresling(plate_radius::Float64, plate_thickness::Float64, plate_mass::Float64, radius::Float64, link_length::Float64, mass::Float64, num_cells::Int)
    # Create empty arrays for plates, short members, long members, and joints
    plates = Body{Float64}[]
    short_members = [Body{Float64}[] for i in 1:num_cells]
    long_members = [Body{Float64}[] for i in 1:num_cells]
    joints = [JointConstraint{Float64}[] for i in 1:num_cells]

    # Create plates
    for i in 1:num_cells+1
        plate = Cylinder(plate_radius, plate_thickness, plate_mass, name=Symbol("plate_$(i)"))
        push!(plates, plate)
    end

    # Create members
    for i in 1:num_cells
        for j in 1:6
            # Create short member
            short_member = Cylinder(radius, link_length, mass, name=Symbol("link$(j)_cell$(i)"))
            push!(short_members[i], short_member)

            # Create long members
            diag1 = Cylinder(radius, diag_length/2, mass, name=Symbol("bot_diag$(j)_cell$(i)"))
            push!(long_members[i], diag1)

            diag2 = Cylinder(radius, diag_length/2, mass, name=Symbol("top_diag$(j)_cell$(i)"))
            push!(long_members[i], diag2)
        end
    end

    # Create joints
    for i in 1:num_cells
        base = plates[i]
        top = plates[i+1]
        bot_start = 0 
        top_start = -pi/3 
        for j in 1:6
            # Create joint for short member
            parent_vertex = [base.shape.rh[1]*cos(pi/3*j), base.shape.rh[1]*sin(pi/3*j), base.shape.rh[2]/2]
            child_vertex = [0, 0, -short_members[i][j].shape.rh[2]/2]
            joint = JointConstraint(Spherical(base, short_members[i][j]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(base.name)_to_$(short_members[i][j].name)"))
            push!(joints[i], joint)

            parent_vertex = [top.shape.rh[1]*cos(pi/3*j), top.shape.rh[1]*sin(pi/3*j), -top.shape.rh[2]/2]
            child_vertex = [0, 0, short_members[i][j].shape.rh[2]/2]
            joint = JointConstraint(Spherical(top, short_members[i][j]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(base.name)_to_$(short_members[i][j].name)"))
            push!(joints[i], joint)

            # Create joint for long member bottom
            diag_bottom = long_members[i][2*j-1]
            bot_θ = bot_start + pi/3*(j-1)
            parent_vertex = [base.shape.rh[1]*cos(bot_θ), base.shape.rh[1]*sin(bot_θ), base.shape.rh[2]/2]
            child_vertex = [0, 0, -diag_bottom.shape.rh[2]/2]
            joint = JointConstraint(Spherical(base, diag_bottom; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(base.name)_to_$(diag_bottom.name)"))
            push!(joints[i], joint)
            
            # Create joint for long member top
            diag_top = long_members[i][2*j]
            top_θ = top_start + pi/3*(j-1)
            parent_vertex = [top.shape.rh[1]*cos(top_θ), top.shape.rh[1]*sin(top_θ), -top.shape.rh[2]/2]
            child_vertex = [0, 0, diag_top.shape.rh[2]/2]
            joint = JointConstraint(Spherical(top, diag_top; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(base.name)_to_$(diag_top.name)"))
            push!(joints[i], joint)

            # Create joint for long member prismatic
            parent_vertex = [0, 0, diag_bottom.shape.rh[2]/2]
            child_vertex = [0, 0, -diag_top.shape.rh[2]/2]
            diag_prismatic = JointConstraint(Prismatic(diag_bottom, diag_top, [0, 0, 1]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(diag_bottom.name)_to_$(diag_top.name)"))
            push!(joints[i], diag_prismatic)
        end
    end

    return plates, short_members, long_members, joints

end

function set_kresling_initial_guess!(mechanism::Mechanism, plate_radius::Float64, plate_thickness::Float64, plate_mass::Float64, radius::Float64, link_length::Float64, mass::Float64, num_cells::Int)
    # Set initial structure guess
    for i in 1:num_cells
        set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(i)")), x=[0, 0, (link_length + plate_thickness) * (i-1)])

        bot_start_angle = 0 
        top_start_angle = -pi/3

        for j in 1:6
            # Set configuration for short member
            set_maximal_configurations!(get_body(mechanism, Symbol("link$(j)_cell$(i)")), x=[plate_radius * cos(pi/3 * j), plate_radius * sin(pi/3 * j), link_length/2 + (link_length + plate_thickness) * (i-1)])

            # Set configuration for long members
            bot_angle = bot_start_angle + pi/3 * (j-1)
            top_angle = top_start_angle + pi/3 * (j-1)

            set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag$(j)_cell$(i)")), x=[plate_radius * cos(bot_angle), plate_radius * sin(bot_angle), diag_length/4 + (link_length + plate_thickness) * (i-1)])

            set_maximal_configurations!(get_body(mechanism, Symbol("top_diag$(j)_cell$(i)")), x=[plate_radius * cos(top_angle), plate_radius * sin(top_angle), diag_length * 3/4 + (link_length + plate_thickness) * (i-1)])
        end
    end
    set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(num_cells+1)")), x=[0, 0, (link_length + plate_thickness) * num_cells])
end

# Link params
# radius = 0.1
# link_length = 1.0
# diag_length = sqrt(2*link_length^2*(1 - cos(deg2rad(80))))
# mass = 0.001

# # Plate params
# plate_radius = 1.0
# plate_thickness = 0.1
# plate_mass = 0.001
# plates, short_members, long_members, joints = get_kresling(plate_radius, plate_thickness, plate_mass, radius, link_length, mass, 3);
# all_bodies = vcat(plates, short_members..., long_members...);
# all_joints = vcat(joints...);
# mechanism = Mechanism(Origin(), all_bodies, all_joints)

# set_kresling_initial_guess!(mechanism, plate_radius, plate_thickness, plate_mass, radius, link_length, mass, 3)

# # Set up simulation
# # Visualizer
# if isdefined(Main, :vis)
#     # If it exists, delete it
#     delete!(vis)
# else
#     # If it doesn't exist, initialize it as a Visualizer
#     vis = Visualizer()
# end
# vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)

# initialize_constraints!(mechanism, 
#     fixedids=[],
#     regularization=0.0,
#     lineIter=10, 
#     newtonIter=20,
#     debug=true,
#     ε =1e-6)

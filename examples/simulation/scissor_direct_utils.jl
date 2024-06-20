using Dojo
using LinearAlgebra
function get_scissor(beam_length::Float64, unit_width::Float64, unit_thickness::Float64, unit_length::Float64, unit_mass::Float64; name_ext::String="scissor", rotation_axis::AbstractArray=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75))
    number_of_units = floor(Int, beam_length/(unit_width))

    get_scissor(number_of_units, unit_thickness, unit_length, unit_mass; name_ext=name_ext, rotation_axis=rotation_axis, color=color)
    
end

function get_scissor(number_of_units::Int64, unit_thickness::Float64, unit_length::Float64, unit_mass::Float64; name_ext::String="scissor", rotation_axis::AbstractArray=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75), add_end_caps::Bool=true)
    # get the number out of name ext if it exists 
    id_offset = 0
    if occursin(r"(\d+)", name_ext)
        id_offset = parse(Int64, match(r"(\d+)", name_ext).captures[1])
    end
    # ### Scissor components
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    
    for i in 1:number_of_units
        body = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i-1)"), color=color)

        body.id = body.id + id_offset*100

        body2 = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i)"), color=color)

        body2.id = body2.id + id_offset*100

        push!(bodies, body)
        push!(bodies, body2)
     
        joint2 = JointConstraint(Revolute(body, body2, rotation_axis), name=Symbol("$(name_ext)_joint_$(2i-1)_to_$(2*i)"))

        push!(joints, joint2)

        if i > 1
            joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[0, 0, -unit_length/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1)-1)_to_$(2*i-1)"))

            joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[0, 0, -unit_length/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1))_to_$(2*i)"))

            push!(joints, joint1)
            push!(joints, joint2)
        end
        
    end

    if add_end_caps
        bodies, joints = add_scissor_ends(unit_thickness, unit_length, unit_mass, bodies, joints; name_ext=name_ext, rotation_axis=rotation_axis, color=color)
    end

    return bodies, joints
end

function add_scissor_ends(unit_thickness, unit_length, unit_mass, bodies, joints; name_ext::String="scissor",  rotation_axis::AbstractArray=[1; 0; 0], color=RGBA(0.75, 0.0, 0.0))
    # left side end cap
    id_offset = 0
    if occursin(r"(\d+)", name_ext)
        id_offset = parse(Int64, match(r"(\d+)", name_ext).captures[1])
    end
    # ### Scissor components
    unit_length /= 2
    new_bodies = Body{Float64}[]
    new_joints = JointConstraint{Float64}[]
    for side in ["left", "right"]
        vertex = side == "left" ? [0, 0, -unit_length/2] : [0, 0, unit_length/2]
        # Create 2 bodies connected at one end with revolute joint and other end connected to scissor bodies
        body = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_$(side)_end_1"), color=RGBA(0.75, 0.0, 0.0))
        body.id = body.id + id_offset*100

        body2 = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_$(side)_end_2"), color=RGBA(0.75, 0.0, 0.0))
        body2.id = body2.id + id_offset*100

        joint1 = JointConstraint(Revolute(body, body2, rotation_axis; parent_vertex=vertex, child_vertex=vertex), name=Symbol("$(name_ext)_joint_$(side)_end_1_to_$(side)_end_2"))

        # Connect to scissor
        parent1 = side == "left" ? bodies[1] : bodies[end]
        parent2 = side == "left" ? bodies[2] : bodies[end-1]
        joint2 = JointConstraint(Revolute(parent1, body, rotation_axis; parent_vertex=2*vertex, child_vertex=-vertex), name=Symbol("$(name_ext)_joint_$(side)_end_1_to_1"))
        joint3 = JointConstraint(Revolute(parent2, body2, rotation_axis; parent_vertex=2*vertex, child_vertex=-vertex), name=Symbol("$(name_ext)_joint_$(side)_end_2_to_2"))

        # Add fixed body to end of add_scissor_ends
        body3 = Sphere(unit_thickness, unit_mass, name=Symbol("$(name_ext)_$(side)_end_fixed"), color=RGBA(0.0, 0.0, 0.75))
        body3.id = body3.id + id_offset*100

        joint4 = JointConstraint(Fixed(body2, body3; parent_vertex=vertex), name=Symbol("$(name_ext)_joint_$(side)_end_2_to_fixed"))

        # Add to bodies and joints
        push!(new_bodies, body)
        push!(new_bodies, body2)
        push!(new_bodies, body3)

        push!(new_joints, joint1)
        push!(new_joints, joint2)
        push!(new_joints, joint3)
        push!(new_joints, joint4)
    
    end
    # Add new_bodies to bodies 
    for body in new_bodies
        push!(bodies, body)
    end
    for joint in new_joints
        push!(joints, joint)
    end

    return bodies, joints
end

function set_scissor_configuration!(mechanism::Mechanism, x::AbstractArray, theta::Float64=pi/4)

    for joint in mechanism.joints
        println(Base.unwrap_unionall(typeof(joint)).parameters[2])
        if Base.unwrap_unionall(typeof(joint)).parameters[2] == 5
            set_minimal_coordinates!(mechanism, joint, [theta])
            return
        end
    end
    println("HERE")
    res = initialize_constraints!(mechanism, 
    fixedids=[mechanism.joints[1].parent_id, mechanism.joints[1].child_id],
    regularization=0.0,
    lineIter=10, 
    newtonIter=20,
    debug=false,
    ε =1e-6)
    println(res)
    return res
end

function move_scissor(mechanism::Mechanism, x::AbstractVector, q::Quaternion, vertex::AbstractVector)
    for body in mechanism.bodies
        # Assuming each body has a 'position' and 'orientation' field
        # Adjust these field names based on your actual structure

        # Rotate the body
        # First, translate the body to the origin based on the vertex
        relative_position = (body.state.x2 + x) - vertex

        # Apply the rotation
        rotated_position = Dojo.vector_rotate(relative_position, q)

        # Translate back from the origin
        position = rotated_position + vertex

        # Update the orientation of the body
        orientation = q * body.state.q2

        # Update the body
        set_maximal_configurations!(body, x=position, q=orientation)
    end
end


# mechanism.joints[end].
# test scissor

# loop from 1 to 10 scissor values and get the difference between rank and size of full_matrix
rank_v_size = []
for i in 1:6
    b, j = get_scissor(i, 0.1, 1.0, 1.0, add_end_caps=false)
    mechanism = Mechanism(Origin(), b, j, timestep=0.01)
    set_scissor_configuration!(mechanism, zeros(3), pi/4)
    # if i > 1
    #         res = initialize_constraints!(mechanism, 
    #                                 fixedids=[mechanism.joints[1].parent_id, mechanism.joints[1].child_id],
    #                                 regularization=0.0,
    #                                 lineIter=10, 
    #                                 newtonIter=10,
    #                                 debug=true,
    #                                 ε = 1e-6)
    # end

    Dojo.set_entries!(mechanism) # compute the residual
    A = full_matrix(mechanism.system)
    F = svd(A, full=true, alg=LinearAlgebra.QRIteration())
    rank_num = sum(F.S .> 1e-6)
    # Plot these values
    push!(rank_v_size, rank_num - size(A, 1))
    # println("Rank: ", rank_num, " Size: ", size(A, 1))
end
using Plots
# increase font size 


plot(1:6, -rank_v_size, label="Singular Values", title="Singular Values of Scissor due to Redundant Constraints", lw=3, legend=:topleft, size=(800, 600), titlefont=(18, "Arial"), xlabel="Number of Scissor Units", ylabel="Number of Singular Values", xtickfont=font(16, "Arial"), ytickfont=font(16, "Arial"), legendfont=font(16, "Arial"), guidefont=font(16, "Arial"))
# plot!(1:6, zeros(6), label="Expected Singular Values", linestyle=:dash)
xlabel!("Number of Scissor Units", )
ylabel!("Number of Singular Values")

rank_v_size

b, j = get_scissor(1, 0.1, 1.0, 1.0, add_end_caps=false)
mechanism = Mechanism(Origin(), b, j, timestep=0.01)
set_scissor_configuration!(mechanism, zeros(3), pi/4)



# # Visualizer
if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)


start_point = get_body(mechanism, Symbol("scissor_left_end_fixed"))
move_scissor(mechanism, -start_point.state.x2, Dojo.RotZ(pi/2), zeros(3))
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)

res = initialize_constraints!(mechanism, 
                                    fixedids=[mechanism.joints[1].parent_id, mechanism.joints[1].child_id],
                                    regularization=0.0,
                                    lineIter=10, 
                                    newtonIter=10,
                                    debug=true,
                                    ε = 1e-6)
Dojo.set_entries!(mechanism) # compute the residual
A = full_matrix(mechanism.system)
F = svd(A, full=true, alg=LinearAlgebra.QRIteration())
rank_num = sum(F.S .> 1e-6)

# start_point.state.q2
# start_point.state.x2
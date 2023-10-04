# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Simulation Parameters
timestep = 0.005
reg = 1e-6

# ### Scissor Parameters
beam_length = 0.266
diag_length = beam_length #sqrt(2*beam_length^2*(1 - cos(deg2rad(80))))
kres_radius = beam_length
rotation_axis = [1; 0; 0]

number_of_units = 5
unit_width = beam_length/number_of_units/2
unit_thickness = beam_length/number_of_units/10
unit_length = 2*unit_width #! fix this

damper = 0.0
unit_mass = 0.01

i = 1
origin = Origin()
plate = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("plate_$(i)"))
joint_con = Cylinder(unit_thickness/2, unit_thickness, unit_mass, name=Symbol("joint_con_$(i)"))
link = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("link_$(i)"))

bodies = [plate, joint_con, link]

fixed_joint = JointConstraint(Fixed(origin, plate), name=Symbol("Fixed_Base_Joint"))

# rot_joint_limits=[Dojo.SA[-pi/2], Dojo.SA[0.1745]]
pitch_joint = JointConstraint(Revolute(plate, joint_con, [1, 0, 0]; parent_vertex=[kres_radius*cos(pi/3*i), kres_radius*sin(pi/3*i), unit_thickness/2], child_vertex=[0, 0, 0], orientation_offset=Dojo.RotZ(pi/3*i)), name=Symbol("pitch_joint"))


# rot_joint_limits=[Dojo.SA[pi/8], Dojo.SA[3pi/4]]
yaw_joint = JointConstraint(Revolute(joint_con, link, [0, 1,0]; parent_vertex=[0, 0, 0], child_vertex=[0, 0, -unit_length/2]),  name=Symbol("yaw_joint"))

joints = [fixed_joint, pitch_joint, yaw_joint]

mechanism = Mechanism(origin, bodies, joints, timestep=timestep)

θ = -7*pi/15 #deg2rad(pi/2)
set_maximal_configurations!(mechanism.bodies[2], x=Dojo.vector_rotate([kres_radius, 0, unit_thickness/2], Dojo.RotZ(pi/3)), q=Dojo.RotZ(pi/3)*Dojo.RotX(θ))
set_maximal_configurations!(mechanism.bodies[3], x=Dojo.vector_rotate([kres_radius, 0.0,unit_thickness/2]+Dojo.vector_rotate([0, 0, unit_length/2], Dojo.RotX(θ)*Dojo.RotY(pi/4)), Dojo.RotZ(pi/3)), q=Dojo.RotZ(pi/3)*Dojo.RotX(θ)*Dojo.RotY(pi/4))


# vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis,visualize_floor=false, show_frame=true)

initialize_constraints!(mechanism, fixedids=[mechanism.bodies[1].id, mechanism.bodies[2].id], regularization=1e-10, lineIter=10, newtonIter=300)
visualize(mechanism, vis=vis,visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.01)

mechanism.gravity=[0.0, 0.0, 0.0]
# ### Simulation
function control!(mechanism, t)
    # nothing
    add_external_force!(mechanism.bodies[3], force=[0, 0, 0.0], torque=[0, 0.01, 0]) # Dojo.vector_rotate([0, 0.1, 0], Dojo.RotZ(pi/3)*Dojo.RotX(pi/4)))
end
opts = SolverOptions(verbose=false, reg=reg, max_iter=25)
@time storage = simulate!(mechanism, 20*mechanism.timestep, control!, record=true, opts=opts)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.001)

#TODO Get Scissors N Units
function get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="scissor", rotation_axis=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75))
    number_of_units = floor(Int, beam_length/(unit_width))

    # ### Scissor components
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    
    for i in 1:number_of_units
        body = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i-1)"), color=color)

        body2 = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i)"), color=color)

        push!(bodies, body)
        push!(bodies, body2)
    
        joint_limits = i % 2 == 0 ? [Dojo.SA[pi/8], Dojo.SA[3pi/4]] : [Dojo.SA[-3pi/4], Dojo.SA[-pi/8]] #[Dojo.SA[-pi/4], Dojo.SA[pi/4]]    
        joint2 = JointConstraint(Revolute(body, body2, rotation_axis, rot_joint_limits=joint_limits), name=Symbol("$(name_ext)_joint_$(2i-1)_to_$(2*i)"))

        push!(joints, joint2)

        if i > 1
            joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[0, 0, -unit_length/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1)-1)_to_$(2*i-1)"))

            joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[0, 0, -unit_length/2]), name=Symbol("$(name_ext)_joint_$(2*(i-1))_to_$(2*i)"))

            push!(joints, joint1)
            push!(joints, joint2)
        end
        
    end

    return bodies, joints
end


#TODO Get Joint Connectors (2*N units (top / bottom))
function get_joint_connectors(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="scissor", rotation_axis=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75))
    # ### Scissor components
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    for i in 1:6
        for side in ["top", "bottom"]
            Cylinder(unit_thickness, unit_thickness, unit_mass, name=Symbol("$(name_ext)_connector_$(side)_$(i)")) 

            # short member
            # left = Cylinder(unit_length, unit_thickness, unit_mass, name=Symbol("$(name_ext)_connector_$(side)_left$(i)")) 
            # push!(bodies, left)
            # right = Cylinder(unit_length, unit_thickness, unit_mass, name=Symbol("$(name_ext)_connector_$(side)_right$(i)")) 
            # push!(bodies, right)

            # joint = JointConstraint(Revolute(left, right, axis; parent_axis=[0, 0, unit_length/2], child_axis=[0, 0, unit_length/2]), name=Symbol("$(name_ext)_connector_joint_$(side)_$(i)"))
            # push!(joints, joint)

        end
    end

    return bodies, joints
end

#TODO Get Plates 
function get_plates(num_cells, kres_radius, unit_thickness, unit_mass)
    plates = Body{Float64}[]
    # Plates
    for i in 1:num_cells+1
        plate = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("plate_$(i)"))

        push!(plates, plate)
    end
    return plates
end

#TODO Connectors to plates 
function joints_to_plates(num_cells, kres_radius, unit_thickness)
    for i in 1:num_cells
        for j in 1:6
            for side in ["top", "bottom"]
                for type in ["short", "long"]
                    if side == "top"
                        plate = plates[i+1]
                    else
                        plate = plates[i]

                    end
                    name ="$(name_ext)_connector_$(side)_$(j)"
                    parent_vertex = [kres_radius*cos(pi/3*(j-1)), kres_radius*sin(pi/3*(j-1)), -unit_thickness]
                    
                    joint = JointConstraint(Revolute(plate, connector, axis, parent_vertex=parent_vertex), name=Symbol("pitch_joint"))

                end
            end
        end
    end

end

#TODO Scissors to plate 
function scissor_to_joints(num_cells, kres_radius, unit_thickness)
    for i in 1:num_cells
        for j in 1:6
            for side in ["top", "bottom"]
                for type in ["short", "long"]
                    if side == "top"
                        plate = plates[i+1]
                    else
                        plate = plates[i]

                    end
                    name ="$(name_ext)_connector_$(side)_$(j)"
                    parent_vertex = [kres_radius*cos(pi/3*(j-1)), kres_radius*sin(pi/3*(j-1)), -unit_thickness]
                    
                    joint = JointConstraint(Revolute(plate, connector, axis, parent_vertex=parent_vertex), name=Symbol("pitch_joint"))

                end
            end
        end
    end

end

#TODO initialize 






# ### Kresling constructor
function get_kresling(kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length, unit_mass; timestep=0.01)
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    # Plates
    for i in 1:num_cells+1
        plate = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("plate_$(i)"))

        push!(plates, plate)
    end

    # Members
    for i in 1:num_cells
        for j in 1:6
            # short member
            body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="short_member$(i)", color=RGBA(1.0, 0.75, 0.75, 0.5))

            # long member
            body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="long_member$(i)", color=RGBA(1.0, 0.75, 0.75, 0.5))
           
        end
    end

    # Joint Members
    for i in 1:num_cells
        
    end
    



    for i in 1:6
        ## Short Member
        body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="short_member$(i)", color=RGBA(1.0, 0.75, 0.75, 0.5))
        # # base_joint = JointConstraint(Fixed(origin, body[1]), name=Symbol("Fixed_Base_Joint"))
        # # push!(joints, base_joint)
        append!(bodies, body)
        append!(joints, joint)
        joint_limits = [Dojo.SA[0.0, 0.0, 0.0], Dojo.SA[0.0, 2pi, 2pi]]
        top_joint = JointConstraint(Spherical(body[end], top; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2],rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Beam_to_Top$(i)"))
        push!(joints, top_joint)

        bottom_joint = JointConstraint(Spherical(base, body[1]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, -unit_length/2], rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Base_to_Beam$(i)"))
        push!(joints, bottom_joint)
        # top_joint = JointConstraint(Orbital(body[end], top, axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2]), name=Symbol("Orbital_Joint_Beam_to_Top$(i)"))
        # push!(joints, top_joint)

        # bottom_joint = JointConstraint(Orbital(body[1], base, axis; parent_vertex=[0, 0, -unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2]), name=Symbol("Orbital_Joint_Base_to_Beam$(i)"))
        # push!(joints, bottom_joint)

        ## Long Member
        body, joint = get_scissor(diag_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="long_member$(i)", color=RGBA(0.75, 1.0, 0.75, 0.5))
        append!(bodies, body)
        append!(joints, joint)
        joint_limits = [Dojo.SA[0.0, 0.0, 0.0], Dojo.SA[0.0, 2pi, 2pi]]
        top_joint = JointConstraint(Spherical(body[end-1], top; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*i), kres_radius*sin(2pi/6*i), -unit_thickness/2], rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Long_Beam_to_Top$(i)"))
        push!(joints, top_joint)

        bottom_joint = JointConstraint(Spherical(base, body[2]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, -unit_length/2],rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Base_to_Long_Beam$(i)"))
        push!(joints, bottom_joint)
        # top_joint = JointConstraint(Orbital(body[end-1], top, axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*i), kres_radius*sin(2pi/6*i), -unit_thickness/2]), name=Symbol("Orbital_Joint_Long_Beam_to_Top$(i)"))
        # push!(joints, top_joint)

        # bottom_joint = JointConstraint(Orbital(body[2],base, axis; parent_vertex=[0, 0, -unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2]), name=Symbol("Orbital_Joint_Base_to_Long_Beam$(i)"))
    end

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)


    return mechanism
end

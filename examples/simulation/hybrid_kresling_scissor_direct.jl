# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Simulation Parameters
timestep = 0.00001

# ### Scissor Parameters
unit_width = 0.006
unit_thickness = 0.006/2
unit_length = 0.045
unit_mass = 0.001
beam_length = 0.266
diag_length = sqrt(2*beam_length^2*(1 - cos(deg2rad(80))))
rotation_axis = [1; 0; 0]
# ### Kresling Parameters
kres_radius = beam_length

damper = 10.0

# ### Scissor constructor 
function get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="scissor", rotation_axis=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75))
    number_of_units = floor(Int, beam_length/(unit_width))
    # number_of_links = 2*number_of_units
    # number_of_joints = 3*number_of_units-4

    # ### Scissor components
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    
    for i in 1:number_of_units
        body = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i-1)"), color=color)

        body2 = Cylinder(unit_thickness, unit_length, unit_mass, name=Symbol("$(name_ext)_link$(2*i)"), color=color)

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

    return bodies, joints
end

# b, j = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="scissor")

# mechanism = Mechanism(Origin(), b, j, timestep=timestep)
# number_of_units = floor(Int, beam_length/(unit_width))

# for i in 1:number_of_units
#     Dojo.set_maximal_configurations!(mechanism.bodies[2i-1], x=Dojo.rotation_matrix(Dojo.RotX(pi/4))*[kres_radius, 0, (i-1)*unit_length*cos(pi/6)], q=Dojo.RotX(pi/4)*Dojo.RotX((-1)^(i+1)*pi/4))

#     Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=Dojo.rotation_matrix(Dojo.RotX(pi/4))*[kres_radius, 0, (i-1)*unit_length*cos(pi/6)], q=Dojo.RotX(pi/4)*Dojo.RotX((-1)^(i)*pi/4))
# end


# ### Kresling constructor
function get_kresling(kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length, unit_mass; timestep=0.01)
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    base = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("base"), color=RGBA(0.75, 0.75, 0.75))
    push!(bodies, base)
    base_joint = JointConstraint(Fixed(origin, base), name=Symbol("Fixed_Base_Joint"))
    push!(joints, base_joint)

    top = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("top"), color=RGBA(0.75, 0.75, 0.75))
    push!(bodies, top)

    for i in 1:6
        ## Short Member
        body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="short_member$(i)", color=RGBA(1.0, 0.0, 0.0, 0.5))
        # # base_joint = JointConstraint(Fixed(origin, body[1]), name=Symbol("Fixed_Base_Joint"))
        # # push!(joints, base_joint)
        append!(bodies, body)
        append!(joints, joint)
        top_joint = JointConstraint(Spherical(body[end], top; parent_vertex=[0, 0, 0], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2]), name=Symbol("Spherical_Joint_Beam_to_Top$(i)"))
        push!(joints, top_joint)

        bottom_joint = JointConstraint(Spherical(base, body[1]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, 0]), name=Symbol("Spherical_Joint_Base_to_Beam$(i)"))
        push!(joints, bottom_joint)

        ## Long Member
        body, joint = get_scissor(diag_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="long_member$(i)", color=RGBA(0.0, 1.0, 0.0, 0.5))
        append!(bodies, body)
        append!(joints, joint)
        top_joint = JointConstraint(Spherical(body[end], top; parent_vertex=[0, 0, 0], child_vertex=[kres_radius*cos(2pi/6*i), kres_radius*sin(2pi/6*i), -unit_thickness/2]), name=Symbol("Spherical_Joint_Long_Beam_to_Top$(i)"))
        push!(joints, top_joint)

        bottom_joint = JointConstraint(Spherical(base, body[1]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, 0]), name=Symbol("Spherical_Joint_Base_to_Long_Beam$(i)"))
        push!(joints, bottom_joint)
    end

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)


    return mechanism
end

function initialize_kresling!(mechanism, kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length)
    set_maximal_configurations!(mechanism.bodies[1], x=[0, 0, 0])
    set_maximal_configurations!(mechanism.bodies[2], x=[0, 0, 1.8*beam_length+unit_thickness], q=Dojo.RotZ(0))
    for i in 1:6
        number_of_units = floor(Int, beam_length/(unit_width))
        # Initialize short members
        for j in 1:number_of_units
            t = (j-1) / (number_of_units-1)

            x = Dojo.rotation_matrix(Dojo.RotZ(0))*[kres_radius*cos((i-1)*pi/3), kres_radius*sin((i-1)*pi/3), (1-t)*unit_thickness/2 + t*(1.8*beam_length+unit_thickness/2)]
            q = Dojo.RotZ((i-1)*pi/3+pi/2)*Dojo.RotX((-1)^(j+1)*pi/4)

            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("short_member$(i)_link$(2*j-1)")), x=x, q=q)

            q = Dojo.RotZ((i-1)*pi/3+pi/2)*Dojo.RotX((-1)^(j)*pi/4)
            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("short_member$(i)_link$(2*j)")), x=x, q=q)
        end

        # Initialize long members
        number_of_units = floor(Int, diag_length/(unit_width))
        for j in 1:number_of_units
            t = (j-1) / (number_of_units-1)
            x = [(1-t)*kres_radius*cos((i-1)*pi/3) + t*kres_radius*cos((i)*pi/3), (1-t)*kres_radius*sin((i-1)*pi/3) + t*kres_radius*sin((i)*pi/3), (1-t)*unit_thickness + t*(2*beam_length+unit_thickness)]
            # x = Dojo.rotation_matrix(Dojo.RotY(pi/4))*([0, 0, (j-1)*unit_width] - [0, 0, number_of_units//2*unit_width]) + [kres_radius*cos((i-1)*pi/3+pi/6), kres_radius*sin((i-1)*pi/3+pi/6), number_of_units//2*unit_width]
            θ = 4pi/6+(i-1)*pi/3 # (1-t)*((i-1)*pi/3+pi/2) + t*((i)*pi/3+pi/2)
            q = Dojo.RotY(-pi/14)*Dojo.RotX(-pi/14)*Dojo.RotZ(θ)*Dojo.RotX((-1)^(j+1)*pi/3)

            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("long_member$(i)_link$(2*j-1)")), x=x, q=q)

            q = Dojo.RotY(-pi/14)*Dojo.RotX(-pi/14)*Dojo.RotZ(θ)*Dojo.RotX((-1)^(j)*pi/3)
            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("long_member$(i)_link$(2*j)")), x=x, q=q)
        end
    end
end

mechanism = get_kresling(kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length, unit_mass; timestep=timestep)

initialize_kresling!(mechanism, kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length)
import Dojo.initialize_constraints!, Dojo.getid, Dojo.joint_residual_violation, Dojo.index_ranges, Dojo.spzeros, Dojo.constraint, Dojo.constraint_jacobian_configuration, Dojo.I, Dojo.VLᵀmat, Dojo.norm, Dojo.constraintstep!, Dojo.violations

fixedidx1 = vcat([(3:4) .+ (i-1)*4 for i in 1:6]...)

fixedidx = vcat([(5:6) .+ fixedidx1[end] .+ (i-1)*6 for i in 1:6]...)

initialize_constraints!(mechanism, fixedids=[mechanism.bodies[1].id, mechanism.bodies[2].id], regularization=1e-6, lineIter=10, newtonIter=100)
# Dojo.set_entries!(mechanism)

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.001)

set_dampers!(mechanism.joints, 100.0)

mechanism.gravity = [0, 0, -9.81]

        
zero_velocities!(mechanism)
# for i in 1:length(mechanism.joints)
#     mechanism.system.matrix_entries[i,i].value += Dojo.I
# end
storage = simulate!(mechanism, 0.1, record=true, opts=SolverOptions(verbose=true))



# vis = visualize_pose(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.0001)

initializeConstraints!(mechanism, freeids = getid.(mechanism.bodies), newtonIter = 500, lineIter = 20)

vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.001)
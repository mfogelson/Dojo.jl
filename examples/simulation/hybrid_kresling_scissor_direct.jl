# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Simulation Parameters
timestep = 0.005
reg = 1e-6

# ### Scissor Parameters
unit_width = 0.266/4
unit_thickness = 0.00266*5
unit_length = 0.25
unit_mass = 0.01
beam_length = 0.266
diag_length = beam_length #sqrt(2*beam_length^2*(1 - cos(deg2rad(80))))
rotation_axis = [1; 0; 0]
# ### Kresling Parameters
kres_radius = beam_length

damper = 10.0
axis = Dojo.SA[1; 1; 0]

# ### Scissor constructor 
function get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="scissor", rotation_axis=[1; 0; 0], color=RGBA(0.75, 0.75, 0.75))
    number_of_units = floor(Int, beam_length/(unit_width))
    println(number_of_units)
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
    
        # joint_limits = i % 2 == 0 ? [Dojo.SA[pi/8], Dojo.SA[3pi/4]] : [Dojo.SA[-3pi/4], Dojo.SA[-pi/8]] #[Dojo.SA[-pi/4], Dojo.SA[pi/4]]    
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
    base_joint = JointConstraint(Spherical(origin, base), name=Symbol("Fixed_Base_Joint"))
    push!(joints, base_joint)

    top = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("top"), color=RGBA(0.75, 0.75, 0.75))
    push!(bodies, top)
    k = 1
    for i in 1:6
        for type in ["short", "long"]
            ## Short Member
            body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="$(type)_member$(i)", color=RGBA(1.0, 0.75, 0.75, 0.5))
            # # base_joint = JointConstraint(Fixed(origin, body[1]), name=Symbol("Fixed_Base_Joint"))
            # # push!(joints, base_joint)
            append!(bodies, body)
            append!(joints, joint)
            # joint_limits = [Dojo.SA[0.0, 0.0, 0.0], Dojo.SA[0.0, 2pi, 2pi]]
            # top_joint = JointConstraint(Spherical(body[end], top; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2],rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Beam_to_Top$(i)"))
            for j in ["top", "bottom"]
                # pitch_joint_part = Cylinder(unit_thickness/2, unit_thickness, unit_mass, name=Symbol("pitch_joint_part_$(type)_$(j)$(i)"), color=RGBA(0.75, 0.75, 0.75, 0.5))

                θ = (type == "long" && j == "top") ? 2pi/6*(i) : 2pi/6*(i-1) 

                plate = j == "top" ? top : base
                dir = j == "top" ? -1 : 1
                # pitch_joint = JointConstraint(Revolute(plate, pitch_joint_part, [1, 0, 0]; parent_vertex=[kres_radius*cos(θ), kres_radius*sin(θ), dir*unit_thickness/2], child_vertex=[0, 0, 0], orientation_offset=Dojo.RotZ(θ)), name=Symbol("pitch_Joint_Beam_$(type)_to_$(j)$(i)")) 
               # A1 A2 have to be co planar and B1 and B2 have to be co planar
               


                if type == "short"
                    if j == "top"
                        ind = length(body)-1
                    else
                        ind = 1
                    end
                else
                    if j == "top"
                        ind = length(body)
                    else
                        ind = 2
                    end
                end
                pitch_joint = JointConstraint(Orbital(plate, body[ind], [1, 1, 0]; parent_vertex=[kres_radius*cos(θ), kres_radius*sin(θ), dir*unit_thickness/2], child_vertex=[0, 0, -dir*unit_length/2], orientation_offset=Dojo.RotZ(θ+pi/2)), name=Symbol("pitch_Joint_Beam_$(type)_to_$(j)$(i)")) 
              

                # yaw_joint = JointConstraint(Revolute(body[ind],pitch_joint_part, [1, 0, 0]; parent_vertex=[0, 0, -dir*unit_length/2], child_vertex=[0, 0, 0]), name=Symbol("yaw_Joint_Beam_$(type)_to_$(j)$(i)")) 
                # yaw_joint = JointConstraint(Revolute(body[ind],pitch_joint_part, [1, 0, 0]; parent_vertex=[0, 0, -dir*unit_length/2], child_vertex=[0, 0, 0]), name=Symbol("yaw_Joint_Beam_$(type)_to_$(j)$(i)")) 

                push!(joints, pitch_joint)
                # push!(joints, yaw_joint)
                # append!(bodies, [pitch_joint_part])
            end


            # bottom_joint = JointConstraint(Spherical(base, body[1]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, -unit_length/2], rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Base_to_Beam$(i)"))
            # push!(joints, bottom_joint)
            # pitch_joint_part = Cylinder(unit_thickness/2, unit_thickness, unit_mass, name=Symbol("pitch_joint_part_bot$(i)"), color=RGBA(0.75, 0.75, 0.75, 0.5))

            # bottom_joint = JointConstraint(Rotational(base, pitch_joint_part, axis=[1, 0, 0]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2], child_vertex=[0, 0, 0], orientation_offset=Dojo.RotZ(2pi/6*(i-1))), name=Symbol("pitch_Joint_Beam_to_Base$(i)")) 
            # push!(joints, bottom_joint)
            # bottom_joint = JointConstraint(Rotational(pitch_joint_part, body[1], axis=[0, 1, 0]; parent_vertex=[0, 0, 0], child_vertex=[0, 0, -unit_length/2]), name=Symbol("yaw_Joint_Beam_to_Base$(i)")) 
            # push!(joints, bottom_joint)
            # append!(bodies, [pitch_joint_part])


            # top_joint = JointConstraint(Orbital(body[end], top, axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), -unit_thickness/2]), name=Symbol("Orbital_Joint_Beam_to_Top$(i)"))
            # push!(joints, top_joint)

            # bottom_joint = JointConstraint(Orbital(body[1], base, axis; parent_vertex=[0, 0, -unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2]), name=Symbol("Orbital_Joint_Base_to_Beam$(i)"))
            # push!(joints, bottom_joint)

            ## Long Member
            # body, joint = get_scissor(diag_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="long_member$(i)", color=RGBA(0.75, 1.0, 0.75, 0.5))
            # append!(bodies, body)
            # append!(joints, joint)
            # # joint_limits = [Dojo.SA[0.0, 0.0, 0.0], Dojo.SA[0.0, 2pi, 2pi]]
            # top_joint = JointConstraint(Spherical(body[end-1], top; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*i), kres_radius*sin(2pi/6*i), -unit_thickness/2], rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Long_Beam_to_Top$(i)"))
            # push!(joints, top_joint)

            # bottom_joint = JointConstraint(Spherical(base, body[2]; parent_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2], child_vertex=[0, 0, -unit_length/2],rot_joint_limits=joint_limits), name=Symbol("Spherical_Joint_Base_to_Long_Beam$(i)"))
            # push!(joints, bottom_joint)
            # top_joint = JointConstraint(Orbital(body[end-1], top, axis; parent_vertex=[0, 0, unit_length/2], child_vertex=[kres_radius*cos(2pi/6*i), kres_radius*sin(2pi/6*i), -unit_thickness/2]), name=Symbol("Orbital_Joint_Long_Beam_to_Top$(i)"))
            # push!(joints, top_joint)

            # bottom_joint = JointConstraint(Orbital(body[2],base, axis; parent_vertex=[0, 0, -unit_length/2], child_vertex=[kres_radius*cos(2pi/6*(i-1)), kres_radius*sin(2pi/6*(i-1)), unit_thickness/2]), name=Symbol("Orbital_Joint_Base_to_Long_Beam$(i)"))
        end
    end

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)


    return mechanism
end

function initialize_kresling!(mechanism, kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length)
    set_maximal_configurations!(mechanism.bodies[1], x=[0, 0, 0])
    height = 4*beam_length
    set_maximal_configurations!(mechanism.bodies[2], x=[0, 0, height/2+unit_thickness], q=Dojo.RotZ(0))
    # for i in 1:6
    #     for type in ["short"]
    #         for j in ["top", "bottom"]
    #             θ = (type == "long" && j == "top") ? 2pi/6*(i) : 2pi/6*(i-1) 
    #             dir = j == "top" ? height/2+unit_thickness/2 : unit_thickness/2
    #             x = [kres_radius*cos(θ), kres_radius*sin(θ),dir]
    #             q = Dojo.RotZ(θ)
    #             # Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("pitch_joint_part_$(type)_$(j)$(i)")), x=x, q=q)
    #         end
    #     end
    # end
    for i in 1:6
        number_of_units = floor(Int, beam_length/(unit_width))
        # Initialize short members
        for j in 1:number_of_units
            t = (j-1) / (number_of_units-1)

            x = Dojo.rotation_matrix(Dojo.RotZ(0))*[(kres_radius+unit_length/2*cos(pi/3))*cos((i-1)*pi/3), (kres_radius+unit_length/2*cos(pi/3))*sin((i-1)*pi/3), (1-t)*unit_thickness/2 + t*(height/2+unit_thickness/2)]
            q = Dojo.RotZ((i-1)*pi/3+pi/2)*Dojo.RotX((-1)^(j+1)*pi/4)

            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("short_member$(i)_link$(2*j-1)")), x=x, q=q)

            q = Dojo.RotZ((i-1)*pi/3+pi/2)*Dojo.RotX((-1)^(j)*pi/4)
            Dojo.set_maximal_configurations!(get_body(mechanism, Symbol("short_member$(i)_link$(2*j)")), x=x, q=q)
        end

        # Initialize long members
        number_of_units = floor(Int, diag_length/(unit_width))
        for j in 1:number_of_units
            t = (j-1) / (number_of_units-1)
            x = [(1-t)*(kres_radius-unit_length/2*cos(pi/3))*cos((i-1)*pi/3) + t*(kres_radius-unit_length/2*cos(pi/3))*cos((i)*pi/3), (1-t)*(kres_radius-unit_length/2*cos(pi/3))*sin((i-1)*pi/3) + t*(kres_radius-unit_length/2*cos(pi/3))*sin((i)*pi/3), (1-t)*unit_thickness + t*(height+unit_thickness/2)]
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

# zero_coordinates!(mechanism)[]
initialize_kresling!(mechanism, kres_radius, beam_length, diag_length, unit_width, unit_thickness, unit_length)

# set_maximal_state!(mechanism, )
vis = Visualizer()
# vis = Visualizer()
delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.001)
# import Dojo.initialize_constraints!, Dojo.getid, Dojo.joint_residual_violation, Dojo.index_ranges, Dojo.spzeros, Dojo.constraint, Dojo.constraint_jacobian_configuration, Dojo.I, Dojo.VLᵀmat, Dojo.norm, Dojo.constraintstep!, Dojo.violations

# fixedidx1 = vcat([(3:4) .+ (i-1)*4 for i in 1:6]...)

# fixedidx = vcat([(5:6) .+ fixedidx1[end] .+ (i-1)*6 for i in 1:6]...)

set_maximal_configurations!(mechanism.bodies[2], x=[0, 0.0, mechanism.bodies[2].state.x2[3]/2], q=Dojo.RotZ(0))
initialize_constraints!(mechanism, fixedids=[mechanism.bodies[1].id], regularization=1e-6, lineIter=10, newtonIter=10)
# Dojo.set_entries!(mechanism)

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.01)

set_dampers!(mechanism.joints, 10.0)

mechanism.gravity = [0, 0, 0.0]

        
zero_velocities!(mechanism)
Fext = -100.0
Text = -100.0
function control!(mechanism, k)
    top = mechanism.bodies[2]
    bottom = mechanism.bodies[1]
    if Dojo.norm(top.state.x2 - bottom.state.x2) < 4*unit_thickness
        zero_velocities!(mechanism)
    else
    # add_external_force!(mechanism.bodies[2], force=[0, 0.0, Fext], torque=[0, 0.0, Text])
        add_external_force!(mechanism.bodies[1], force=[0, 0, 1.0], torque=[0, 0, -0.0])
    end

end
# for i in 1:length(mechanism.joints)
#     mechanism.system.matrix_entries[i,i].value += Dojo.I
# end
opts = SolverOptions(verbose=false, reg=reg, max_iter=25)
@time storage = simulate!(mechanism, 1000*mechanism.timestep, control!, record=true, opts=opts)

vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.001)
# @save "hybrid_kresling_1_cell_close_explosion_sim_$(Fext)_Fext_$(Text)_Text.jld2" mechanism storage


unction set_maximal_state!(mechanism::Mechanism, storage::Storage; ind=1)
    for (body, x, q, ω, v) in zip(mechanism.bodies, storage.x, storage.q, storage.ω, storage.v)
        body.state.x1 = x[ind]
        body.state.x2 = x[ind]
        body.state.q1 = q[ind]
        body.state.q2 = q[ind]
        body.state.ω15 = ω[ind]
        body.state.v15 = v[ind]
        body.state.d -= body.state.d
        body.state.D -=  body.state.D
        body.state.Fext -= body.state.Fext
        body.state.τext -= body.state.τext
        body.state.vsol[1] -= body.state.vsol[1] 
        body.state.vsol[2] -= body.state.vsol[2]
        body.state.ωsol[1] -= body.state.ωsol[1]
        body.state.ωsol[2] -= body.state.ωsol[2]
    end
end
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.001)
set_maximal_state!(mechanism, storage, ind=1)
# state = get_maximal_state(mechanism)

cur_state = get_maximal_state(mechanism)
@save "hybrid_kresling_1_cell_initial_state_guess.jld2" mechanism cur_state

collapsed_state = load("hybrid_kresling_1_cell_initial_state_guess.jld2")["cur_state"]

Dojo.set_maximal_state!(mechanism, cur_state)

min_state = get_minimal_state(mechanism)
off = 0
for body in mechanism.bodies
    # x2, v15, q2, ω15 = Dojo.unpack_data(collapsed_state[Dojo.SUnitRange(off+1,end)]); off += 13
    x2 = collapsed_state[off+1:off+3]
    v15 = collapsed_state[off+4:off+6]
    q2 = collapsed_state[off+7:off+10]
    ω15 = collapsed_state[off+11:off+13]
    off += 13
    q2 = Quaternion(q2...)
    body.state.v15 = v15
    body.state.ω15 = ω15
    body.state.x2 = x2
    body.state.q2 = q2
end
Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2

for body in mechanism.bodies
    Dojo.set_velocity_solution!(body)
end

function get_joint_state(mechanism::Mechanism{T,Nn,Ne,Nb,Ni}, joint::JointConstraint) where {T,Nn,Ne,Nb,Ni} 
    c = zeros(T,0)
    v = zeros(T,0)
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    for element in (joint.translational, joint.rotational)
        pos = minimal_coordinates(element, pbody, cbody)
        vel = minimal_velocities(element, pbody, cbody, timestep)
        push!(c, pos...)
        push!(v, vel...)
    end

    return [c; v]
end


for joint in mechanism.joints
    if occursin("short", String(joint.name)) || occursin("long", String(joint.name))
        println(joint.name, get_joint_state(mechanism, joint))
    end
end
get_joint_state(mechanism, mechanism.joints[2])
# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Parameters
radius = 0.1
link_length = 1.0
diag_length = sqrt(2*link_length^2*(1 - cos(deg2rad(80))))
mass = 0.001
rotation_axis = [1;0;0] 
connection = [0;0;0]
damper = 1.0
timestep = 0.005
reg = 1e-7

num_cells = 10


# ### Mechanism components
function get_kresling()
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    loop_joints = JointConstraint{Float64}[]
    n = 1
    top = Cylinder(link_length, radius, mass, name=Symbol(n%2==1 ? "top_plate_$(n)" : "bottom_plate_$(n)"))
    push!(bodies, top)
    for n in 1:num_cells
        # create base and top plates
        base = Cylinder(link_length, radius, mass, name=Symbol(n%2==0 ? "top_plate_$(n)" : "bottom_plate_$(n)"))
        push!(bodies, base)
        if n == 1
            base_joint = JointConstraint(Fixed(origin, base))
            push!(joints, base_joint)
        
        end

  
        # if n == 2
        #     top_joint = JointConstraint(Fixed(origin, top, parent_vertex=[0, 0, link_length+radius]))
        #     push!(joints, top_joint)
        # end

        # links 
        for i in 1:6 
            # short member
            body = Cylinder(radius, link_length, mass, name=Symbol("link$(i)_$(n)"))
            push!(bodies, body)
            joint_bottom = JointConstraint(Spherical(base, body; parent_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), (-1)^(n+1)*radius/2], child_vertex=[0, 0, (-1)^(n)*link_length/2]), name=Symbol("joint_$(base.name)_to_$(body.name)"))
            push!(joints, joint_bottom)

            if n == 1
                joint_top = JointConstraint(Spherical(body, top; parent_vertex=[0, 0, (-1)^(n+1)*link_length/2], child_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), (-1)^(n)*radius/2]), name=Symbol("joint_$(body.name)_to_$(top.name)"))
                push!(loop_joints, joint_top)
            else
                joint_top = JointConstraint(Spherical(top, body; child_vertex=[0, 0, (-1)^(n+1)*link_length/2], parent_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), (-1)^(n)*radius/2]), name=Symbol("joint_$(top.name)_to_$(body.name)"))
                push!(loop_joints, joint_top)
            end 

            # long member
            diag_top = Cylinder(radius, diag_length/2, mass, name=Symbol(n%2==1 ? "top_diag$(i)_$(n)" : "bot_diag$(i)_$(n)"))
            push!(bodies, diag_top)

            if n == 1
                joint_top = JointConstraint(Spherical(diag_top, top; parent_vertex=[0, 0, (-1)^(n+1)*diag_length/4], child_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), (-1)^(n)*radius/2]), name = Symbol("joint_$(diag_top.name)_to_$(top.name)"))
                push!(joints, joint_top)
            else
                joint_top = JointConstraint(Spherical(top, diag_top; child_vertex=[0, 0, (-1)^(n+1)*diag_length/4], parent_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), (-1)^(n)*radius/2]), name = Symbol("joint_$(top.name)_to_$(diag_top.name)"))
                push!(joints, joint_top)
            end

            diag_bottom = Cylinder(radius, diag_length/2, mass, name=Symbol(n%2==0 ? "top_diag$(i)_$(n)" : "bot_diag$(i)_$(n)"))
            push!(bodies, diag_bottom)

            if n == 1
                joint_bottom = JointConstraint(Spherical(base, diag_bottom; parent_vertex=[link_length*cos(2pi/6*(i+1)), link_length*sin(2pi/6*(i+1)), (-1)^(n+1)*radius/2], child_vertex=[0, 0, (-1)^(n)*diag_length/4]), name = Symbol("joint_$(base.name)_to_$(diag_bottom.name)"))
                push!(joints, joint_bottom)
            else
                joint_bottom = JointConstraint(Spherical(diag_bottom, base; child_vertex=[link_length*cos(2pi/6*(i+1)), link_length*sin(2pi/6*(i+1)), (-1)^(n+1)*radius/2], parent_vertex=[0, 0, (-1)^(n)*diag_length/4]), name = Symbol("joint_$(diag_bottom.name)_to_$(base.name)"))
                push!(joints, joint_bottom)
            end

            if n == 1
                diag_prismatic = JointConstraint(Prismatic(diag_bottom, diag_top, [0, 0, 1]; parent_vertex=[0, 0, (-1)^(n+1)*diag_length/4], child_vertex=[0, 0, (-1)^(n)*diag_length/4]), name=Symbol("joint_$(diag_bottom.name)_to_$(diag_top.name)"))
                push!(loop_joints, diag_prismatic)
            else
                diag_prismatic = JointConstraint(Prismatic(diag_top, diag_bottom, [0, 0, 1]; parent_vertex=[0, 0, (-1)^(n)*diag_length/4], child_vertex=[0, 0, (-1)^(n+1)*diag_length/4]), name=Symbol("joint_$(diag_top.name)_to_$(diag_bottom.name)"))
                push!(loop_joints, diag_prismatic)
            end
        end
    end

    # ### Construct Mechanism
    append!(joints, loop_joints)
    # connect cells
    # combo = JointConstraint(Fixed(bodies[2], bodies[22]), name=Symbol("joint_$(bodies[2].name)_to_$(bodies[22].name)"))
    # push!(joints, combo)
    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)
    set_dampers!(mechanism.joints, damper)
    return mechanism
end

function reflect_rigid_body!(body, plane_normal, plane_center)
    # Normalize the plane normal vector
    plane_normal = plane_normal / Dojo.norm(plane_normal)

    # Translate the position vector relative to the plane's center
    relative_position = body.state.x2 - plane_center
    
    # Reflect the position vector about the plane
    reflected_relative_position = relative_position - 2 * Dojo.dot(relative_position, plane_normal) * plane_normal
    
    # Translate the reflected position back to the original coordinate system
    reflected_position = reflected_relative_position + plane_center
    
    # Reflect the attitude quaternion about the plane
    reflected_attitude_quaternion = body.state.q2

    set_maximal_configurations!(body, x=reflected_position, q=reflected_attitude_quaternion)
    
    return nothing
end

function initialize_kresling!(mechanism)
    for n in 1:num_cells
        # set maximal configurations for plates
        set_maximal_configurations!(get_body(mechanism, Symbol(n%2==0 ? "top_plate_$(n)" : "bottom_plate_$(n)")), x=[0, 0, 0])

        if n == 1
            set_maximal_configurations!(get_body(mechanism, Symbol(n%2==1 ? "top_plate_$(n)" : "bottom_plate_$(n)")), x=[0, 0, link_length+radius])
        end
    

        # set maximal configurations for links 1
        set_maximal_configurations!(get_body(mechanism, Symbol("link1_$n")), x=[link_length*cos(pi/3), link_length*sin(pi/3), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag1_$n")), x=[link_length*cos(pi/3), link_length*sin(pi/3), radius/2+3link_length/4], q=Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag1_$n")), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+1link_length/4], q=Dojo.RotY(pi/4))


        # set maximal configurations for links 2
        set_maximal_configurations!(get_body(mechanism, Symbol("link2_$n")), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag2_$n")), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+3link_length/4], q=Dojo.RotZ(pi/3)*Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag2_$n")), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+1link_length/4], q=Dojo.RotZ(pi/3)*Dojo.RotY(pi/4))

        # set maximal configurations for links 3
        set_maximal_configurations!(get_body(mechanism, Symbol("link3_$n")), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag3_$n")), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+3link_length/4], q=Dojo.RotZ(2pi/3)*Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag3_$n")), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+1link_length/4], q=Dojo.RotZ(2pi/3)*Dojo.RotY(pi/4))

        # set maximal configurations for links 4
        set_maximal_configurations!(get_body(mechanism, Symbol("link4_$n")), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag4_$n")), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+3link_length/4], q=Dojo.RotZ(3pi/3)*Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag4_$n")), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+1link_length/4], q=Dojo.RotZ(3pi/3)*Dojo.RotY(pi/4))

        # set maximal configurations for links 5
        set_maximal_configurations!(get_body(mechanism, Symbol("link5_$n")), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag5_$n")), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+3link_length/4], q=Dojo.RotZ(4pi/3)*Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag5_$n")), x=[link_length*cos(6pi/3), link_length*sin(6pi/3), radius/2+1link_length/4], q=Dojo.RotZ(4pi/3)*Dojo.RotY(pi/4))

        # set maximal configurations for links 6
        set_maximal_configurations!(get_body(mechanism, Symbol("link6_$(n)")), x=[link_length*cos(2pi), link_length*sin(2pi), radius/2+link_length/2])
        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag6_$(n)")), x=[link_length*cos(6pi/3), link_length*sin(6pi/3), radius/2+3link_length/4], q=Dojo.RotZ(5pi/3)*Dojo.RotY(pi/4))
        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag6_$(n)")), x=[link_length*cos(7pi/3), link_length*sin(7pi/3), radius/2+1link_length/4], q=Dojo.RotZ(5pi/3)*Dojo.RotY(pi/4))
    end

    # reflect cell 2 about x-y plane at midpoint of cell 1 topplate
    for i in 1:19
        reflect_rigid_body!(mechanism.bodies[i+20], [0, 0, 1], [0, 0, (link_length+radius)])
    end

    for b in mechanism.bodies
        Dojo.set_previous_configuration!(b, mechanism.timestep)
    end

end

# function clear_bodies!(mechanism::Mechanism{T}) where T
#     for body in mechanism.bodies
#         body.state.x1 = zeros(3)
#         body.state.x2 = zeros(3)
#         body.state.v15 = zeros(3)
#         body.state.q1 = Quaternion(1, 0, 0, 0)
#         body.state.q2 = Quaternion(1, 0, 0, 0)
#         body.state.ω15 = zeros(3)
#         body.state.vsol[1] = szeros(T, 3)
#         body.state.ωsol[1] = szeros(T, 3)
#     end
# end
# clear_bodies!(mechanism)
mechanism = get_kresling()

initialize_kresling!(mechanism)


if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)

initialize_constraints!(mechanism, fixedids=[mechanism.bodies[1].id,mechanism.bodies[2].id, mechanism.bodies[21].id], regularization=1, lineIter=10, newtonIter=300) #,  mechanism.bodies[21].id, mechanism.bodies[22].id]

for b in mechanism.bodies
    Dojo.set_previous_configuration!(b, mechanism.timestep)
end

Dojo.set_entries!(mechanism, reg=reg)

Dojo.residual_violation(mechanism)

# [Dojo.joint_residual_violation(mechanism, j) for j in mechanism.joints]
# body_constraint = [Dojo.norm(Dojo.constraint(mechanism, b), Inf) for b in mechanism.bodies]

# for id in Dojo.connections(mechanism.system, mechanism.bodies[2].id)
#     Dojo.impulses!(mechanism, body, get_node(mechanism, id))
# end

τext = 1.0*[0, 0, -1.0]
# set_external_force!(mechanism.bodies[2], force=0.0*τext, torque=-1.0*τext)
set_external_force!(mechanism.bodies[21], force=0.0*τext, torque=-τext)

set_external_force!(mechanism.bodies[1], force=0.0*τext, torque=2.0*τext)
# set_external_force!(mechanism.bodies[22], torque=τext)


# set_minimal_velocities!(mechanism, joints[1], [0.0])
# set_minimal_velocities!(mechanism, joints[2], [0.0])
# set_minimal_velocities!(mechanism, joint12, [0.2])

mechanism.gravity = [0, 0, -9.81]


function control!(mechanism, k)
    # if Dojo.norm(mechanism.bodies[1].state.x2-mechanism.bodies[2].state.x2) < 4*radius
    #     zero_velocities!(mechanism)
    #     # set_external_force!(mechanism.bodies[21], torque=τext)
    #     # set_external_force!(mechanism.bodies[22], torque=-1.0τext)

    #     # set_external_force!(mechanism.bodies[2], torque=-1.0*τext)
    #     # τext .*= -1

    #     # set_external_force!(mechanism.bodies[1], torque=-1.0*τext)
    #     # set_minimal_velocities!(mechanism,
    # end
    # set_external_force!(mechanism.bodies[2], force=[0, 0, 10])
    # set_external_force!(mechanism.bodies[2], force=0.0*τext, torque=-1.0*τext)
    # set_external_force!(mechanism.bodies[22], torque=τext)

    set_external_force!(mechanism.bodies[21], force=0.0*τext, torque=-τext)
    set_external_force!(mechanism.bodies[1], force=0.0*τext, torque=2.0*τext)

    return
end
# ### Simulate
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_joint=true, show_frame=true)
steps = 2000
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=false, rtol=1e-6, reg=reg)
simulate!(mechanism, 1:steps, storage, control!, record=true, opts=opts)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_joint=true, show_frame=true)

prev_storage = storage

storage
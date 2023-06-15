# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Parameters
radius = 0.1
link_length = 1
diag_length = sqrt(2*link_length^2*(1 - cos(deg2rad(80))))
mass = 0.001
rotation_axis = [1;0;0] 
connection = [0;0;0]
damper = 1.0

num_sets = 8

# ### Mechanism components
origin = Origin()
bodies = Body{Float64}[]
joints = JointConstraint{Float64}[]
loop_joints = JointConstraint{Float64}[]
base = Cylinder(link_length, radius, mass, name=Symbol("base"))
push!(bodies, base)
base_joint = JointConstraint(Fixed(origin, base))
push!(joints, base_joint)

top = Cylinder(link_length, radius, mass, name=Symbol("top"))
push!(bodies, top)

# fixed_base = JointConstraint(Fixed(base, origin))

# links 
for i in 1:6 
    body = Cylinder(radius, link_length, mass, name=Symbol("link$(i)"))
    push!(bodies, body)
    joint_bottom = JointConstraint(Spherical(base, body; parent_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), radius/2], child_vertex=[0, 0, -link_length/2]))
    joint_top = JointConstraint(Spherical(body, top; parent_vertex=[0, 0, link_length/2], child_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), -radius/2]))
    push!(joints, joint_bottom)
    push!(loop_joints, joint_top)

    diag_top = Cylinder(radius, diag_length/2, mass, name=Symbol("top_diag$(i)"))
    push!(bodies, diag_top)
    joint_top = JointConstraint(Spherical(diag_top, top; parent_vertex=[0, 0, diag_length/4], child_vertex=[link_length*cos(2pi/6*i), link_length*sin(2pi/6*i), -radius/2]))
    push!(joints, joint_top)


    diag_bottom = Cylinder(radius, diag_length/2, mass, name=Symbol("bot_diag$(i)"))
    push!(bodies, diag_bottom)
    joint_bottom = JointConstraint(Spherical(base, diag_bottom; parent_vertex=[link_length*cos(2pi/6*(i+1)), link_length*sin(2pi/6*(i+1)), radius/2], child_vertex=[0, 0, -diag_length/4]))
    push!(joints, joint_bottom)

    diag_prismatic = JointConstraint(Prismatic(diag_bottom, diag_top, [0, 0, 1]; parent_vertex=[0, 0, -diag_length/4], child_vertex=[0, 0, -diag_length/4], ))
    push!(loop_joints, diag_prismatic)
end


# bodies = Body{Float64}[]
# joints = JointConstraint{Float64}[]
# loop_joints = JointConstraint{Float64}[]
# for i in 1:num_sets
#     body = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i-1)"))
#     body2 = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i)"))

#     push!(bodies, body)
#     push!(bodies, body2)

#     if i == 1
#         joint1 = JointConstraint(Revolute(origin, body, rotation_axis))
#         joint2 = JointConstraint(Revolute(body, body2, rotation_axis))
#         push!(joints, joint1)
#         push!(joints, joint2)
#     else
#         joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
#         joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
#         push!(joints, joint1)
#         push!(joints, joint2)

#         joint3 = JointConstraint(Revolute(body, body2, rotation_axis))
#         push!(loop_joints, joint3)
#     end
# end


# ### Construct Mechanism
append!(joints, loop_joints)
mechanism = Mechanism(origin, bodies, joints, timestep=0.001)

set_dampers!(mechanism.joints, damper)

exlude_ids = [j.id for j in loop_joints]
# ### Set state
# for i in 1:num_sets
#     set_minimal_coordinates!(mechanism, joints[2*i-1], [(-1)^(i+1)*pi/4], exclude_ids=exlude_ids)
#     set_minimal_coordinates!(mechanism, joints[2*i], [(-1)^(i)*pi/4], exclude_ids=exlude_ids)
# end
# set_minimal_coordinates!(mechanism, joints[1], [pi/3], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[2], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[3], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[4], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[5], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[6], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[7], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[8], [pi/2], exclude_ids=exlude_ids)
set_maximal_configurations!(mechanism.bodies[1], x=[0, 0, 0])
set_maximal_configurations!(mechanism.bodies[2], x=[0, 0, link_length+radius], q=Dojo.RotZ(0))
set_maximal_configurations!(get_body(mechanism, :link1), x=[link_length*cos(pi/3), link_length*sin(pi/3), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag1), x=[link_length*cos(pi/3), link_length*sin(pi/3), radius/2+3link_length/4], q=Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag1), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+1link_length/4], q=Dojo.RotY(pi/4))


set_maximal_configurations!(get_body(mechanism, :link2), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag2), x=[link_length*cos(2pi/3), link_length*sin(2pi/3), radius/2+3link_length/4], q=Dojo.RotZ(pi/3)*Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag2), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+1link_length/4], q=Dojo.RotZ(pi/3)*Dojo.RotY(pi/4))

set_maximal_configurations!(get_body(mechanism, :link3), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag3), x=[link_length*cos(3pi/3), link_length*sin(3pi/3), radius/2+3link_length/4], q=Dojo.RotZ(2pi/3)*Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag3), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+1link_length/4], q=Dojo.RotZ(2pi/3)*Dojo.RotY(pi/4))

set_maximal_configurations!(get_body(mechanism, :link4), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag4), x=[link_length*cos(4pi/3), link_length*sin(4pi/3), radius/2+3link_length/4], q=Dojo.RotZ(3pi/3)*Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag4), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+1link_length/4], q=Dojo.RotZ(3pi/3)*Dojo.RotY(pi/4))

set_maximal_configurations!(get_body(mechanism, :link5), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag5), x=[link_length*cos(5pi/3), link_length*sin(5pi/3), radius/2+3link_length/4], q=Dojo.RotZ(4pi/3)*Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag5), x=[link_length*cos(6pi/3), link_length*sin(6pi/3), radius/2+1link_length/4], q=Dojo.RotZ(4pi/3)*Dojo.RotY(pi/4))

set_maximal_configurations!(get_body(mechanism, :link6), x=[link_length*cos(2pi), link_length*sin(2pi), radius/2+link_length/2])
set_maximal_configurations!(get_body(mechanism, :top_diag6), x=[link_length*cos(6pi/3), link_length*sin(6pi/3), radius/2+3link_length/4], q=Dojo.RotZ(5pi/3)*Dojo.RotY(pi/4))
set_maximal_configurations!(get_body(mechanism, :bot_diag6), x=[link_length*cos(7pi/3), link_length*sin(7pi/3), radius/2+1link_length/4], q=Dojo.RotZ(5pi/3)*Dojo.RotY(pi/4))


τext = -10*[0, 0, -1]
set_external_force!(mechanism.bodies[1], torque=-1.0*τext)

set_external_force!(mechanism.bodies[2], torque=τext)
# set_minimal_velocities!(mechanism, joints[1], [0.0])
# set_minimal_velocities!(mechanism, joints[2], [0.0])
# set_minimal_velocities!(mechanism, joint12, [0.2])

mechanism.gravity = [0, 0, -9.81]

function control!(mechanism, k)
    if Dojo.norm(mechanism.bodies[1].state.x2-mechanism.bodies[2].state.x2) < 4*radius
        zero_velocities!(mechanism)
        τext .*= -1

        # set_external_force!(mechanism.bodies[1], torque=-1.0*τext)
        # set_minimal_velocities!(mechanism,
    end
    set_external_force!(mechanism.bodies[2], force=[0, 0, 10])


    return
end
# ### Simulate
storage = simulate!(mechanism, 1.0, control!, record=true)

# ### Visualize
delete!(vis)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_joint=true, show_frame=true)
render(vis)
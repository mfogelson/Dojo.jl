# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Parameters
radius = 0.03
link_length = 0.45 # m? 
mass = 0.0015 # kg? 
rotation_axis = [1;0;0] 
connection = [0;0;0]

num_sets = 3

# ### Mechanism components
origin = Origin()

bodies = Body{Float64}[]
joints = JointConstraint{Float64}[]
loop_joints = JointConstraint{Float64}[]
for i in 1:num_sets
    body = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i-1)"))
    body2 = Cylinder(radius, link_length, mass, name=Symbol("link$(2*i)"))

    push!(bodies, body)
    push!(bodies, body2)

    if i == 1
        joint1 = JointConstraint(Revolute(origin, body, rotation_axis))
        joint2 = JointConstraint(Revolute(body, body2, rotation_axis))
        push!(joints, joint1)
        push!(joints, joint2)
    else
        joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
        joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
        push!(joints, joint1)
        push!(joints, joint2)

        joint3 = JointConstraint(Revolute(body, body2, rotation_axis))
        push!(loop_joints, joint3)
    end
end


# ### Construct Mechanism
append!(joints, loop_joints)
mechanism = Mechanism(origin, bodies, joints, timestep=0.00001)
set_dampers!(mechanism.joints, 0.001)

exlude_ids = [j.id for j in loop_joints]
# ### Set state
# for i in 1:num_sets
#     set_minimal_coordinates!(mechanism, joints[2*i-1], [(-1)^(i+1)*pi/4], exclude_ids=exlude_ids)
#     set_minimal_coordinates!(mechanism, joints[2*i], [(-1)^(i)*pi/4], exclude_ids=exlude_ids)
# end

for i in 1:num_sets
    Dojo.set_maximal_configurations!(mechanism.bodies[2i-1], x=[0, 0, (i-1)*link_length*cos(pi/4)], q=Dojo.RotX((-1)^(i+1)*pi/4))
    Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=[0, 0, (i-1)*link_length*cos(pi/4)], q=Dojo.RotX((-1)^(i)*pi/4))
end

# set_minimal_coordinates!(mechanism, joints[1], [pi/4], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[2], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[3], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[4], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[5], [pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[6], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[7], [-pi/2], exclude_ids=exlude_ids)
# set_minimal_coordinates!(mechanism, joints[8], [pi/2], exclude_ids=exlude_ids)


# set_minimal_velocities!(mechanism, joints[1], [0.5])
# set_minimal_velocities!(mechanism, joints[2], [-0.5])
zero_velocities!(mechanism)
# set_maximal_velocities!(mechanism.bodies[1], ω=[50.0;0;0])
# set_maximal_velocities!(mechanism.bodies[2], ω=[-50.0;0;0])

add_external_force!(mechanism.bodies[1], force=[0;0;0], torque=[10.0;0;0], vertex=[0;0;0])
add_external_force!(mechanism.bodies[2], force=[0;0;0], torque=[-10.0;0;0], vertex=[0;0;0])


# # set_minimal_velocities!(mechanism, joint12, [0.2])

# mechanism.gravity = [0;0;9.8]

# ### Simulate
# Regularization
# for i in 1:mechanism.system.matrix_entries.n
#     mechanism.system.matrix_entries[i,i].value += Dojo.I*1e-6
# end

storage = simulate!(mechanism, 0.5, record=true, opts=SolverOptions(verbose=true))

# ### Visualize
# vis = Visualizer()
delete!(vis)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)
render(vis)
# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Parameters
radius = 0.1
link_length = 1.0
mass = 0.001
rotation_axis = [0;1;0] 
damper = 1.0
reg = 1e-10

# ### Make triangle
function direct_4_bar()
    timestep = 0.0001

    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    length_1 = 1.2
    length_2 = 1.4
    length_3 = 0.4
    length_4 = 1.0
    link1 = Cylinder(radius, length_1, mass, name=Symbol("link1"), color=RGBA(1.0, 0, 0))
    link2 = Cylinder(radius, length_2, mass, name=Symbol("link2"), color=RGBA(0, 1.0, 0))
    link3 = Cylinder(radius, length_3, mass, name=Symbol("link3"), color=RGBA(0, 0, 1.0))

    push!(bodies, link1)
    push!(bodies, link2)
    push!(bodies, link3)

    joint1 = JointConstraint(Revolute(origin, link1, rotation_axis; parent_vertex=[0, 0, 0], child_vertex=[0, 0, -length_1/2]), name=Symbol("joint1"))
    joint2 = JointConstraint(Revolute(link1, link2, rotation_axis; parent_vertex=[0, 0, length_1/2], child_vertex=[0, 0, -length_2/2]), name=Symbol("joint2"))
    joint3 = JointConstraint(Revolute(link3, link2, rotation_axis; parent_vertex=[0, 0, -length_3/2], child_vertex=[0, 0, length_2/2]), name=Symbol("joint3"))
    loop_joint = JointConstraint(Revolute(origin, link3, rotation_axis; parent_vertex=[-length_4, 0, 0], child_vertex=[0, 0, length_3/2]), name=Symbol("loop_joint"))

    push!(joints, joint1)
    push!(joints, joint2)
    push!(joints, joint3)
    push!(joints, loop_joint)

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)
end

# ## initialize configurations
function initialize_4_bar!(mechanism)
    offset = 0
    θ = -pi/3
    Dojo.set_maximal_configurations!(mechanism.bodies[1], x=Dojo.vector_rotate([0, 0, -link_length], Dojo.RotY(offset)), q=Dojo.RotY(pi)*Dojo.RotY(offset))
    Dojo.set_maximal_configurations!(mechanism.bodies[2], x=Dojo.vector_rotate([-link_length/2, 0, -link_length], Dojo.RotY(offset)), q=Dojo.RotY(-pi/2)*Dojo.RotY(offset))
    Dojo.set_maximal_configurations!(mechanism.bodies[3], x=Dojo.vector_rotate([-link_length, 0, -link_length/2], Dojo.RotY(offset)), q=Dojo.RotY(0)*Dojo.RotY(offset))

    initialize_constraints!(mechanism, fixedids=[], regularization=1e-6, lineIter=10, newtonIter=100)
end

mechanism = direct_4_bar()

initialize_4_bar!(mechanism)

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)
mechanism.gravity = [0, 0, -9.81]

opts = SolverOptions(verbose=false, reg=reg, max_iter=100)
tf = 1.0
function ctrl!(mech, t)
    # if t > 1
    #     mat = full_matrix(mech.system)[21:end, 1:18]
    #     Dojo.rank(mat) != 17 ? println(t, Dojo.rank(mat)) : nothing
    #     s = Dojo.svd(mat).S
    #     # println(s)
    #     # println("Minimum: $(minimum(s))")
    #     # println("Maximum: $(maximum(s))")
    # end
    if Dojo.norm(mech.bodies[3].state.v15) > 1.5
        # println("Velocity too high")
        return nothing
    end
    Dojo.set_input!(mech, 1.0 * Dojo.SVector(0, 0, -0.01, 0))
    return nothing
end
storage = Dojo.simulate!(mechanism, tf, ctrl!, record=true, opts=opts)
delete!(vis)
vis = visualize(mechanism, storage, vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)
render(vis)


data_matrices = []
angles = []
function get_joint_jacobian(mechanism)
    Dojo.jacobian_joint_data!(mechanism.data_matrix, mechanism)
    nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]

    dimrow = length.(nodes)
    dimcol = Dojo.data_dim.(nodes)
    datajac1 = Dojo.full_matrix(mechanism.data_matrix, false, dimrow, dimcol)
    return datajac1[19:end, 1:18]
end

push!(angles, mechanism.bodies[1].state.q2)
joint_jac = get_joint_jacobian(mechanism)
push!(data_matrices, joint_jac)

num_steps = 100
for i in 1:num_steps
    cur_state = get_maximal_state(mechanism)
    cur_input = zero(cur_state)
    next_state = step!(mechanism, cur_state, cur_input, opts=opts)
    push!(angles, mechanism.bodies[1].state.q2)
    joint_jac = get_joint_jacobian(mechanism)
    push!(data_matrices, joint_jac)
end

angles[1]-angles[end]
data_matrices[1]-data_matrices[end]


other_joint_jac = []
for body in mechanism.bodies
    push!(other_joint_jac, Dojo.constraint_jacobian_configuration(mechanism, body))
end
# @time storage = simulate!(mechanism, 1.0, record=true, opts=opts)
# vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.001)

# data_matrix = Dojo.create_data_matrix(mechanism.joints, mechanism.bodies, mechanism.contacts)
# Dojo.jacobian_joint_data!(data_matrix, mechanism)
# nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]

# dimrow = length.(nodes)
# dimcol = Dojo.data_dim.(nodes)
# datajac1 = Dojo.full_matrix(data_matrix, false, dimrow, dimcol)

# collect(data_matrix)

# Dojo.jacobian_data!(data_matrix, mechanism)

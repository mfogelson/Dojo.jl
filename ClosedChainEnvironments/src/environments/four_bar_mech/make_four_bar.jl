# ### Setup
# PKG_SETUP
using Dojo
using LinearAlgebra
# ### Parameters
radius = 0.0254 # m 
link_length = 1.0
density = 500 # kg/m^3
mass = 0.001
rotation_axis = [0;1;0] 
damper = 1.0
reg = 1e-10

# ### Make triangle
function direct_4_bar()
    timestep = 0.01

    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    length_1 = 0.1016 # 4 in
    length_2 = 0.3556 # 14 in
    length_3 = 0.3048 # 12 in
    length_4 = 0.254 # 10 in
    mass = density * pi * radius^2 * length_1
    link1 = Cylinder(radius, length_1, mass, name=Symbol("link1"), color=RGBA(1.0, 0, 0))

    mass = density * pi * radius^2 * length_2
    link2 = Cylinder(radius, length_2, mass, name=Symbol("link2"), color=RGBA(0, 1.0, 0))

    mass = density * pi * radius^2 * length_3
    link3 = Cylinder(radius, length_3, mass, name=Symbol("link3"), color=RGBA(0, 0, 1.0))

    push!(bodies, link1)
    push!(bodies, link2)
    push!(bodies, link3)

    # joint1 = JointConstraint(Revolute(origin, link1, rotation_axis; parent_vertex=[0, 0, 0], child_vertex=[0, 0, -length_1/2]), name=Symbol("joint1"))
    # joint2 = JointConstraint(Revolute(link1, link2, rotation_axis; parent_vertex=[0, 0, length_1/2], child_vertex=[0, 0, -length_2/2]), name=Symbol("joint2"))
    # joint3 = JointConstraint(Revolute(link3, link2, rotation_axis; parent_vertex=[0, 0, -length_3/2], child_vertex=[0, 0, length_2/2]), name=Symbol("joint3"))
    # loop_joint = JointConstraint(Revolute(origin, link3, rotation_axis; parent_vertex=[-length_4, 0, 0], child_vertex=[0, 0, length_3/2]), name=Symbol("loop_joint"))
    tra_joint_limits = [szeros(Float64,0), szeros(Float64,0)] # [[-1.25e-5, -1.25e-5], [1.25e-5, 1.25e-5]] #[[-0.0, -0.0], [0.0, 0.0]]
    # rotational_limits =  [[0.0, -20*pi, -0.0], [0.0, 20*pi, 0.0]] #[Dojo.SA[-pi, -pi, -pi], Dojo.SA[pi, pi, pi]]#[szeros(Float64,0), szeros(Float64,0)]  #[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    joint1 = JointConstraint(Revolute(origin, link1, rotation_axis; child_vertex=[0, 0, -length_1/2], damper=0.1), name=Symbol("joint1"))

    joint2 = JointConstraint(Revolute(link1, link2, rotation_axis; parent_vertex=[0, 0, length_1/2], child_vertex=[0, 0, -length_2/2]), name=Symbol("joint2"))

    joint3 = JointConstraint(Revolute(link3, link2, rotation_axis; parent_vertex=[0, 0, -length_3/2], child_vertex=[0, 0, length_2/2]), name=Symbol("joint3"))

    loop_joint = JointConstraint(Revolute(origin, link3, rotation_axis; parent_vertex=[-length_4, 0, 0], child_vertex=[0, 0, length_3/2]), name=Symbol("loop_joint"))

    push!(joints, joint1)
    push!(joints, joint2)
    push!(joints, joint3)
    push!(joints, loop_joint)

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)
    freeids = Set(Dojo.getid.(mechanism.bodies))
    z0 = get_maximal_configuration(mechanism, freeids) #.+ 1e-5
    z0 = rand(length(z0))
    set_configuration!(mechanism, freeids, z0)
    for body in mechanism.bodies
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
        body.state.x2 = [body.state.x2[1], body.state.x2[2], 0.0]
    end


    newtonIter = 200
    storage = Storage(newtonIter, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=fixed_ids, newtonIter = newtonIter, lineIter = 10, ε = 1e-8, debug=true, storage=storage)
    z0_revolute = deepcopy(get_maximal_state(mechanism))

    ## With joint limits
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    length_1 = 0.1116 # 4 in
    length_2 = 0.3556 # 14 in
    length_3 = 0.3048 # 12 in
    length_4 = 0.254 # 10 in
    mass = density * pi * radius^2 * length_1
    link1 = Cylinder(radius, length_1, mass, name=Symbol("link1"), color=RGBA(1.0, 0, 0))

    mass = density * pi * radius^2 * length_2
    link2 = Cylinder(radius, length_2, mass, name=Symbol("link2"), color=RGBA(0, 1.0, 0))

    mass = density * pi * radius^2 * length_3
    link3 = Cylinder(radius, length_3, mass, name=Symbol("link3"), color=RGBA(0, 0, 1.0))

    push!(bodies, link1)
    push!(bodies, link2)
    push!(bodies, link3)

    
    tra_joint_limits = [[-1.25e-2, -1.25e-2], [1.25e-2, 1.25e-2]] 

    joint1 = JointConstraint(Revolute(origin, link1, rotation_axis; child_vertex=[0, 0, -length_1/2], damper=0.1), name=Symbol("joint1"))

    joint2 = JointConstraint(PlanarAxis(link1, link2, rotation_axis; parent_vertex=[0, 0, length_1/2], child_vertex=[0, 0, -length_2/2], tra_joint_limits=tra_joint_limits), name=Symbol("joint2"))

    joint3 = JointConstraint(PlanarAxis(link3, link2, rotation_axis; parent_vertex=[0, 0, -length_3/2], child_vertex=[0, 0, length_2/2], tra_joint_limits=tra_joint_limits), name=Symbol("joint3"))

    loop_joint = JointConstraint(PlanarAxis(origin, link3, rotation_axis; parent_vertex=[-length_4, 0, 0], child_vertex=[0, 0, length_3/2], tra_joint_limits=tra_joint_limits), name=Symbol("loop_joint"))

    push!(joints, joint1)
    push!(joints, joint2)
    push!(joints, joint3)
    push!(joints, loop_joint)

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep)
    set_maximal_state!(mechanism, z0_revolute)

    return mechanism

end

mechanism = direct_4_bar()


if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.1)
# mechanism.gravity = [0, 0, 0.0]

opts = SolverOptions(verbose=true, rtol=1e-6, btol=1e-6,reg=1e-10, max_iter=50)
tf = mechanism.timestep*500
print_angle(mechanism, joint) = println("Joint Angle: $(Dojo.minimal_coordinates(mechanism, joint))")
function ctrl!(mech, t)
    # zero_velocities!(mechanism)

    print_angle(mechanism, mechanism.joints[1])
    # if t > 1
    #     mat = full_matrix(mech.system)[21:end, 1:18]
    #     Dojo.rank(mat) != 17 ? println(t, Dojo.rank(mat)) : nothing
    #     s = Dojo.svd(mat).S
    #     # println(s)
    #     # println("Minimum: $(minimum(s))")
    #     # println("Maximum: $(maximum(s))")
    # end
    if Dojo.norm(mech.bodies[1].state.v15) > 1.5
        # println("Velocity too high")
        return nothing
    end
    set_input!(get_joint(mechanism, Symbol("joint1")), [-1.0])
    # Dojo.set_input!(mech, 1.0 * Dojo.SVector(0, 0, -0.01, 0))
    return nothing
end
zero_velocities!(mechanism)
# mechanism.joints[2].rotational.joint_limits = [Dojo.SA[-pi, -pi, -pi], Dojo.SA[pi, pi, pi]]
# mechanism.joints[3].rotational.joint_limits = [Dojo.SA[-pi, -pi, -pi], Dojo.SA[pi, pi, pi]]
# mechanism.joints[4].rotational.joint_limits = [Dojo.SA[-pi, -pi, -pi], Dojo.SA[pi, pi, pi]]
storage = Dojo.simulate!(mechanism, tf, ctrl!, record=true, opts=opts)
# delete!(vis)
vis = visualize(mechanism, storage, vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.1)

storage_e3 = deepcopy(storage)
# Plot the position, velocity and accelerationi of each body in the mechanism
using Plot
num_bodies = length(mechanism.bodies)
num_steps = length(storage.x[1])
# Extract x, y, z coordinates for plotting
xs = [[storage.x[i][j][1] for j in 1:num_steps] for i in 1:num_bodies]
ys = [[storage.x[i][j][2] for j in 1:num_steps] for i in 1:num_bodies]
zs = [[storage.x[i][j][3] for j in 1:num_steps] for i in 1:num_bodies]

xs_e3 = [[storage_e3.x[i][j][1] for j in 1:num_steps] for i in 1:num_bodies]
ys_e3 = [[storage_e3.x[i][j][2] for j in 1:num_steps] for i in 1:num_bodies]
zs_e3 = [[storage_e3.x[i][j][3] for j in 1:num_steps] for i in 1:num_bodies]



# Create the 3D scatter plot
scatter()
for i in 1:num_bodies
    # get error between xs and xs_e3
    xerr = Dojo.norm(xs[i] - xs_e3[i])
    yerr = Dojo.norm(ys[i] - ys_e3[i])
    zerr = Dojo.norm(zs[i] - zs_e3[i])
    println("Body $i e3 error: $xerr, $yerr, $zerr")
    # scatter!(xerr, yerr, zerr, label="Body $i e3")
    # scatter!(xs[i], ys[i], zs[i], label="Body $i e5")
end
scatter!(xlabel="X", ylabel="Y", zlabel="Z", title="3D Position of Bodies Over Steps")
# add is so I can rotate 




plot(pe_00[10:end], label="Potential Energy Slop=0.0", xlabel="Time", ylabel="Energy", title="Energy vs Time", lw=2)
plot!(pe_01[10:end], label="Potential Energy Slop=0.1", lw=2)
plot!(pe_001[10:end], label="Potential Energy Slop=0.01", lw=2)


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

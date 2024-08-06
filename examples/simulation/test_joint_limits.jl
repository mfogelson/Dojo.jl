using Dojo

# ### Parameters
radius = 0.1
link_length = 1.0
mass = 0.001
rotation_axis = [0;1;0] 
damper = 1.0
reg = 1e-10
length_1 = 1.2
timestep = 0.0001

function get_mech()
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    link1 = Cylinder(radius, length_1, mass, name=Symbol("link1"), color=RGBA(1.0, 0, 0))
    push!(bodies, link1)

    translation_limits = [[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]] #[[-0.0, -0.0], [0.0, 0.0]]
    rotational_limits =  [[-0.01, -20*pi, -0.01], [0.01, 20*pi, 0.01]] #[Dojo.SA[-pi, -pi, -pi], Dojo.SA[pi, pi, pi]]#[szeros(Float64,0), szeros(Float64,0)]  #[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    # joint1 = JointConstraint(Free(origin, link1, rotation_axis; rot_joint_limits=rotational_limits), name=Symbol("joint1"))
    joint1 = JointConstraint(Floating(origin, link1; tra_joint_limits=translation_limits, rot_joint_limits=rotational_limits), name=Symbol("joint1"))

    # joints[1] = joint1
    push!(joints, joint1)

    mechanism = Mechanism(origin, bodies, joints, timestep=timestep, gravity=[0.0, 0.0, -0.021])
end

mechanism = get_mech()
Dojo.set_maximal_configurations!(mechanism.bodies[1], x=[0, 0, 0.001], q=Dojo.RotY(0))

opts = SolverOptions(verbose=true, rtol=1e-3, btol=1e-3, reg=1e-3, max_iter=20)
tf = mechanism.timestep*1
storage = Dojo.simulate!(mechanism, tf, record=true, opts=opts)

vis = Visualizer()
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)
using Dojo

peg = Cylinder(0.003, 0.045, 0.0015, name=:peg)
constraint_peg = Cylinder(0.023, 0.045, 0.0015, name=:constraint_peg)
origin = Origin()

joint1 = JointConstraint(PlanarFree(origin, peg, [0; 0; 1], tra_joint_limits=[[-0.01, -0.01], [0.01, 0.01]], child_vertex=[0, 0, 0.045/2], parent_vertex=[0, 0, 0.045]), name=:origin_joint)
# joint2 = JointConstraint(PlanarFree(origin, peg, [0; 0; 1], tra_joint_limits=[[-0.01, -0.01], [0.01, 0.01]], child_vertex=[0, 0, -0.45/2]), name=:peg_joint)
# joint1 = JointConstraint(Spherical(origin, peg, rot_joint_limits=[[-pi, -pi, -pi], [pi, pi, pi]]), name=:origin_joint)

bodies = Body{Float64}[peg]
joints = JointConstraint{Float64}[joint1] #, joint2]

mechanism = Mechanism(origin, bodies, joints, gravity=[-9.81, 0, 0], timestep=0.01)

vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis["body"], visualize_floor=false)
 # create arrow
constraint_shape = Dojo.convert_shape(constraint_peg.shape)
Dojo.setobject!(vis["constraint"], constraint_shape, Dojo.MeshPhongMaterial(color=RGBA(0.5, 0.5, 0.5, 0.5)))

# move joint through its range
# Cycle through joint workspace

function set_configurations!(mechanism, name, x, q)
    body = get_body(mechanism, name)
    set_maximal_configurations!(body, x=x, q=q)
end


set_configurations!(mechanism, :peg, [0.0, 0.0, 0.00], Dojo.RotY(0.0))
# add_external_force!(mechanism.bodies[1], torque=[1.0, 0.0, 0.0], vertex=[0.0, 0.0, 0.0])
# set_input!(mechanism.joints[1], [0.0, 0.0, 0.00001, 0.0, 0.0])
steps = 100
storage = Storage(steps, length(mechanism.bodies))

simulate!(mechanism, 1:steps, storage,
            record=true, 
            opts=SolverOptions(rtol=1e-5, btol=1e-6, reg=1e-6, verbose=true, svd_threshold=1e-6),
            abort_upon_failure=false,
            solver=Dojo.mehrotra_svd!)

visualize(mechanism, storage, vis=vis["body"], visualize_floor=false, show_frame=true)
            
    z_next = Dojo.step!(mechanism, get_maximal_state(mechanism), zeros(Dojo.input_dimension(mechanism)))
visualize(mechanism, vis=vis["body"], visualize_floor=false)

z0 = get_maximal_state(mechanism)
Dojo.set_maximal_state!(mechanism, z0)  
Dojo.set_entries!(mechanism)
bvio = Dojo.bilinear_violation(mechanism) # does not require to apply set_entries!
rvio = Dojo.residual_violation(mechanism) # does not require to apply set_entries!
Dojo.pull_residual!(mechanism)               # store the residual inside mechanism.residual_entries

A = Dojo.full_matrix(mechanism.system)
b = Dojo.full_vector(mechanism.system)

z_next = Dojo.step!(mechanism, get_maximal_state(mechanism), zeros(Dojo.input_dimension(mechanism)))
Dojo.initialize_state!(mechanism)
Dojo.set_entries!(mechanism)
for joint in mechanism.joints
    Dojo.input_impulse!(joint, mechanism)
    println(joint.name)
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    println(cbody.name)
    origin_parent = pbody.state.x2+Dojo.vector_rotate( joint.translational.vertices[1], pbody.state.q2)
    origin_child = cbody.state.x2+Dojo.vector_rotate(joint.translational.vertices[2], cbody.state.q2)
    if pbody.name != Symbol("origin")
        parent_forces = Dojo.impulse_map(mechanism, joint, pbody)*joint.impulses[2]
        Dojo.set_arrow!(vis, origin_parent, Dojo.vector_rotate(parent_forces[1:3], pbody.state.q2), scaling=0.1, color=RGBA(1, 0, 0, 0.5), name=pbody.name)
    end
    child_forces = Dojo.impulse_map(mechanism, joint, cbody)*joint.impulses[2]
    println(Dojo.impulse_map(mechanism, joint, cbody))
    println(origin_child)
    println(child_forces)
    Dojo.set_arrow!(vis, origin_child, Dojo.vector_rotate(child_forces[1:3], cbody.state.q2), shaft_radius=0.0005, max_head_radius=0.005, scaling=10.0, color=RGBA(0, 1, 0, 0.5), name=joint.name)
end
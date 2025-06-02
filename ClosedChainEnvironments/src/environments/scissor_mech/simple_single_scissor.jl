using Dojo
using LinearAlgebra

include("scissor_mechanism.jl")
include("controllers.jl")

length_scale = 100.0 # cm 
time_scale = 100.0 # cs 
mass_scale = 100.0 # cg 

link_length = 0.045 * length_scale
link_mass = 0.0015 * mass_scale
link_radius = 0.003 * length_scale
damper = 0.001 * (length_scale/time_scale^2)
gravity = [9.81*(length_scale/time_scale^2), 0.0, 0.0]
timestep = 0.001*time_scale

mechanism = get_scissor_mechanism(num_sets=3, initial_angle=-pi/3, link_length=link_length, link_mass=link_mass, link_radius=link_radius, damper=damper, gravity=gravity, timestep=timestep)

steps = 1:100
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(undercut=10.0, verbose=true)
simulate!(mechanism, steps, storage, angle_controller!, opts=opts, verbose=true, solver=Dojo.mehrotra_svd!)

vis = Visualizer()
visualize(mechanism, storage, vis=vis, visualize_floor=false)

function step(mechanism, z, control!, opts)
    set_maximal_state!(mechanism, z)
    Dojo.initialize_simulation!(mechanism)
    control!(mechanism, 0)
    for joint in mechanism.joints Dojo.input_impulse!(joint, mechanism) end
    status = mehrotra!(mechanism, opts=opts) #mehrotra!(mechanism, opts=opts)
    for body in mechanism.bodies Dojo.clear_external_force!(body) end
    for body in mechanism.bodies Dojo.update_state!(body, mechanism.timestep) end

    return Dojo.get_next_state(mechanism)
end

function objective(data, storage, timesteps)
    mechanism = get_scissor_mechanism(num_sets=3, initial_angle=-pi/3, link_length=link_length, link_mass=link_mass, link_radius=link_radius, damper=data, gravity=gravity, timestep=timestep)

    # simulate!(mechanism, timesteps, storage, angle_controller!, opts=opts, verbose=true, solver=Dojo.mehrotra_svd!)
    cost = 0.0
    state_predicted = get_maximal_state(storage, 1)
    Q = Diagonal(ones(length(state_predicted)))

    for i in timesteps
        state_true = get_maximal_state(storage, i+1)
        state_predicted = step(mechanism, state_predicted, angle_controller!, opts)

        cost += 0.5 * (state_predicted-state_true)'*Q*(state_predicted-state_true)
    end
    return cost
end

objective(damper, storage, steps)

Dojo.initialize_simulation!(mechanism)
Dojo.set_entries!(mechanism) # compute the residual
nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]

dimrow = length.(nodes)
dimcol = Dojo.data_dim.(nodes)

Dojo.jacobian_joint_data!(mechanism.data_matrix, mechanism)

full_data_jac = Dojo.full_matrix(mechanism.data_matrix, false, dimrow, dimcol)
full_data_jac[:, 1:6]
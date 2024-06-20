# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

timestep = 0.001
reg = 1e-8
mech = get_mechanism(:fourbar;
    timestep,
    parse_springs=false,
    parse_dampers=false,
    dampers=0.0)
Dojo.initialize!(mech, :fourbar,
    inner_angle=0.25)
loopjoints = mech.joints[end:end]
Dojo.root_to_leaves_ordering(mech) == [2, 7, 3, 6, 1, 8, 4, 9]



initialize_constraints!(mech, fixedids=[mech.bodies[1]], regularization=1e-6, lineIter=10, newtonIter=100)

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# delete!(vis)
vis = visualize(mech; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)
mechanism.gravity = [0, 0, -9.81]


# Simulation
function ctrl!(mech, t)
    Dojo.rank(full_matrix(mech.system)[26:end, 1:24]) != 22 ? println(Dojo.rank(full_matrix(mech.system)[26:end, 1:24])) : nothing

    Dojo.set_input!(mech, 1.0 * Dojo.SVector(rand(), -rand(), 0.0, 0.0, 0.0))
    return nothing
end

opts = SolverOptions(verbose=false, reg=reg, max_iter=100)

# data_matrices = []
# angles = []
# function get_joint_jacobian(mechanism)
#     Dojo.jacobian_joint_data!(mechanism.data_matrix, mechanism)
#     nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]

#     dimrow = length.(nodes)
#     dimcol = Dojo.data_dim.(nodes)
#     datajac1 = Dojo.full_matrix(mechanism.data_matrix, false, dimrow, dimcol)
#     return datajac1[19:end, 1:18]
# end

# push!(angles, mech.bodies[1].state.q2)
# joint_jac = get_joint_jacobian(mech)
# push!(data_matrices, joint_jac)

# num_steps = 1000
# for i in 1:num_steps
#     print(Dojo.rank(full_matrix(mech.system)[26:end, 1:24]) != 22 ? Dojo.rank(full_matrix(mech.system)[26:end, 1:24]) : "")
#     cur_state = get_maximal_state(mech)
#     cur_input = [rand(), -rand(), 0.0, 0.0, 0.0]
#     next_state = step!(mech, cur_state, cur_input, opts=opts)
#     push!(angles, mech.bodies[1].state.q2)
#     joint_jac = get_joint_jacobian(mech)
#     push!(data_matrices, joint_jac)
# end
# vis = visualize(mech; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)

storage = Dojo.simulate!(mech, 5.0, record=true, opts=opts)
delete!(vis)
vis = visualize(mech, storage, vis=vis)
render(vis)

min_coords = Dojo.get_minimal_coordinates(mech)
Dojo.norm(min_coords[5] - +min_coords[4], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - -min_coords[3], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - (min_coords[2] - min_coords[1]), Inf) < 1.0e-5

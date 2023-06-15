# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Get mechanism (check DojoEnvironment/mechanisms files for kwargs)
mechanism = get_mechanism(:triscissor, contact=false, gravity=0.0, radius=0.05, len=1.0, scale=1.0, num_sets=1) 

# build_robot
vis = Visualizer()
delete!(vis)
# Dojo.build_robot(mechanism, vis=vis, show_joint=true, show_frame=true, visualize_floor=false)

# ### Initialize mechanism (check DojoEnvironment/mechanisms files for kwargs)[]
# initialize!(mechanism, :triscissor)

# ### Simulate mechanism
storage = simulate!(mechanism, 0.01, record=true)
    
# ### Visualize mechanism
vis = visualize(mechanism, storage, vis=vis, show_joint=true, show_frame=true, visualize_floor=false)
render(vis)


# for timestep in [0.10, 0.05, 0.01, 0.005]
mech = get_mechanism(:fourbar;
    timestep=0.01,
    parse_springs=false,
    parse_dampers=false,
    dampers=0.0)
Dojo.initialize!(mech, :fourbar,
    inner_angle=0.25)
loopjoints = mech.joints[end:end]
Dojo.root_to_leaves_ordering(mech) == [2, 7, 3, 6, 1, 8, 4, 9]

# Simulation
function ctrl!(m, t)
    Dojo.set_input!(m, 1.0 * Dojo.SVector(rand(), -rand(), 0.0, 0.0, 0.0))
    return nothing
end
storage = Dojo.simulate!(mech, 5.0, ctrl!, verbose=false, record=true)

min_coords = Dojo.get_minimal_coordinates(mech)
Dojo.norm(min_coords[5] - +min_coords[4], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - -min_coords[3], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - (min_coords[2] - min_coords[1]), Inf) < 1.0e-5
# end
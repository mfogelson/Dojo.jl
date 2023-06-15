# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

timestep = 0.001
mech = get_mechanism(:fourbar;
    timestep,
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
storage = Dojo.simulate!(mech, 5.0, record=true)
delete!(vis)
vis = visualize(mech, storage, vis=vis)
render(vis)

min_coords = Dojo.get_minimal_coordinates(mech)
Dojo.norm(min_coords[5] - +min_coords[4], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - -min_coords[3], Inf) < 1.0e-5
Dojo.norm(min_coords[5] - (min_coords[2] - min_coords[1]), Inf) < 1.0e-5

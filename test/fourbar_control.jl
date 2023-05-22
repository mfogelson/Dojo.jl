using Dojo
vis = Visualizer()
# for timestep in [0.10, 0.05, 0.01, 0.005]
mech = get_mechanism(:fourbar;
    model="fourbar",
    timestep=0.005,
    damper=1.0)
Dojo.initialize!(mech, :fourbar,
    angle=0.25)
build_robot(mech, vis=vis)
loopjoints = mech.joints[end:end]
Dojo.root_to_leaves_ordering(mech) == [2, 7, 3, 6, 1, 8, 4, 9]
get_minimal_coordinates(mech)
for (key, val) in get_minimal_coordinates(mech)
    println(key, get_joint(mech, key).name)
end
# Simulation
function ctrl!(m, t)
    set_input!(get_joint(m, 4), 100.0*SVector(rand()))
    # Dojo.set_input!(m, 100.0 * SVector(-rand(), rand(), 0.0, 0.0, 0.0))
    return nothing
end
storage = Dojo.simulate!(mech, 5.0, ctrl!, verbose=false, record=true)
delete!(vis)
visualize(mech, storage, vis=vis)
min_coords = Dojo.get_minimal_coordinates(mech)
for j in mech.joints
    println(j.name)
end
# @test norm(min_coords[5] - +min_coords[4], Inf) < 1.0e-5
# @test norm(min_coords[5] - -min_coords[3], Inf) < 1.0e-5
# @test norm(min_coords[5] - (min_coords[2] - min_coords[1]), Inf) < 1.0e-5
# end
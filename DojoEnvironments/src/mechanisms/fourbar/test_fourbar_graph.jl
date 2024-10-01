using Dojo
timestep = 0.01
gravity = [0.0; 0.0; -9.81]
model = :fourbar
spring = 0.0
damper = 0.0
parse_damper = true
T = Float64

path = joinpath(@__DIR__, "dependencies/$(String(model)).urdf")
mech = Mechanism(path; floating=false, T,
    gravity,
    timestep,)

for joint in mech.joints
    println(joint.name, joint.parent_id, joint.child_id, joint.translational.vertices)
end

get_minimal_state(mech)

vis = Visualizer()
delete!(vis)
visualize(mech, vis=vis, visualize_floor=false, show_frame=false)
# Dojo.build_robot(mech, vis=vis)
Dojo.zero_velocities!(mech)
a = 0.45
get_joint(mech, :jointb1).minimal_index
av = szeros(2)
Dojo.set_minimal_coordinates_velocities!(mech, get_joint(mech, :jointb1);
    xmin=[-a, -a])
Dojo.set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint12);
    xmin=[+2a, 0])
Dojo.set_minimal_coordinates_velocities!(mech, get_joint(mech, :jointb3);
    xmin=[+a, -a])
Dojo.set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint34);
    xmin=[-2a, 0])
z = get_maximal_state(mech)
Dojo.get_minimal_coordinates(mech)
# Dojo.set_robot(vis, mech, z)
visualize(mech, vis=vis, visualize_floor=false, show_frame=false)
using DojoEnvironments
DojoEnvironments.initialize!(mech, "fourbar")
# delete!(vis)
z = get_maximal_state(mech)

# Simulation
function ctrl!(m, t)
    Dojo.set_input!(m, 1.0 * SVector(rand(), -rand(), 0.0, 0.0, 0.0))
    return nothing
end
storage = Dojo.simulate!(mech, 20*mech.timestep, ctrl!, verbose=true, record=true)

# set_robot(vis, mech, z)
# Adding springs and dampers
for timestep in [0.10, 0.05, 0.01, 0.005]
    loopjoints = mech.joints[end:end]
    Dojo.root_to_leaves_ordering(mech) == [2, 7, 3, 6, 1, 8, 4, 9]

    # Simulation
    function ctrl!(m, t)
        Dojo.set_input!(m, 1.0 * SVector(rand(), -rand(), 0.0, 0.0, 0.0))
        return nothing
    end
    storage = Dojo.simulate!(mech, 5.0, ctrl!, verbose=false, record=true)
end
visualize(mech, storage, vis=vis, visualize_floor=false, show_frame=true)
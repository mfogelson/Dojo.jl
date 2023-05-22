using Dojo
timestep = 0.01
gravity = [0.0; 0.0; -9.81]
model = :fourbar
spring = 0.0
damper = 0.0
parse_damper = true
T = Float64

path = joinpath(@__DIR__, "../deps/$(String(model)).urdf")
mech = Mechanism(path; floating=false, T,
    gravity,
    timestep,
    parse_damper)

for joint in mech.joints
    println(joint.name, joint.parent_id, joint.child_id, joint.translational.vertices)
end

get_minimal_state(mech)

vis = Visualizer()
build_robot(mech, vis=vis)
zero_velocity!(mech)
a = 0.45
get_joint(mech, :jointb1).minimal_index
av = szeros(2)
set_minimal_coordinates_velocities!(mech, get_joint(mech, :jointb1);
    xmin=[-a, -a])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint12);
    xmin=[+2a, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :jointb3);
    xmin=[+a, -a])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint34);
    xmin=[-2a, 0])
z = get_maximal_state(mech)
get_minimal_coordinates(mech)
set_robot(vis, mech, z)
using DojoEnvironments
DojoEnvironments.initialize!(mech, "fourbar", angle=pi / 4)
# delete!(vis)
z = get_maximal_state(mech)
set_robot(vis, mech, z)
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
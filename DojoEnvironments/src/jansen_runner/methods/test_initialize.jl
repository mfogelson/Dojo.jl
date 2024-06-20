# Variables
using Dojo
timestep=0.01
gravity=[0.0; 0.0; -9.81]
friction_coefficient=1.0
contact_foot=false
contact_body=false
limits=false
model=:strandbeest
floating=false
contact_type=:linear #non-linear
spring=0.0
damper=0.0
parse_damper=true
T=Float64

# Model 
path = joinpath(@__DIR__, "../deps/Jansen.urdf")
mech = Mechanism(path; floating, T,
gravity,
timestep,
parse_damper)
# for b in mech.bodies
#     b.mass = 1.0
#     # b.inertia = I(3)
#     # println(b.inertia)
# end

vis = Visualizer()
delete!(vis)
build_robot(mech, vis=vis)
# get_joint(mech, 4).rotational.joint_limits
# Adding springs and dampers
using DojoEnvironments
DojoEnvironments.set_springs!(mech.joints, spring)
DojoEnvironments.set_dampers!(mech.joints, damper)

for j in mech.joints
    println(j.name)
end
# Bar M
# 0.0
set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint_crossbar_crank); xmin=[pi, 0.1])

# Bar J
# 5.07361093803733
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_m_j); xmin=[5.07361093803733, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_m_j); xmin=[5.07361093803733, 0])

# Bar B
# 4.368635601032737
# Bar E
# 3.568982565099022
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_e); xmin=[-0.4273899115092289, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_e); xmin=[-0.4273899115092289, 0])

# Bar K
# 3.9677042625400483
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_k); xmin=[-0.8261116089502555-pi/11, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_k); xmin=[-0.8261116089502555-pi/11, 0])
# Bar C
# 1.8557540393718421
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_c); xmin=[-4.7, 0])#997346692961635, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_c); xmin=[-4.7, 0])#997346692961635, 0])
# Bar F
# 5.118452698935833
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_e_f); xmin=[-1.8557540393718421, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_e_f); xmin=[-1.8557540393718421, 0])
# Bar I 
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_i); xmin=[-0.5, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_i); xmin=[-0.5, 0])

set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint_crossbar_crank); xmin=[pi/2, 0])

# vis = Visualizer()
delete!(vis)
build_robot(mech, vis=vis, show_joint=false)
z = get_maximal_state(mech)
set_robot(vis, mech, z)

get_joint(mech, :joint_crossbar_crank).translational
function controller!(mech, t)

    x = get_minimal_state(mech)
    x_goal = zero(similar(x))
    # x_goal = get_minimal_state(mech)
    # x_goal[7] = -pi/2
    # x_goal[11] = -pi/2

    # x_goal[2] = -0.2
    # x_goal[7] = 0.0
    K = fill!(similar(x), 100.0)
    u =@. -K * (x - x_goal)
    # println(u)
    set_input!(mech, u)
end
for x in mech.joints
    println(x.name)
end
# get_minimal_state(mech)
for (key, val) in get_minimal_coordinates(mech)
    println(key)
    println(get_joint(mech, key).name)
    # println(get_joint(mech, key).translational)

end

for z in get_minimal_state(mech)
    println(z)
end
# z = get_minimal_coordinates(mech)
# z[13]
# z[8] = [1.57]
# set_minimal_coordinates!(mech, z)
# get_joint(mech, 7).rotational.joint_limits

# set_minimal_coordinates!(mech, get_joint(mech, :pair01_joint_crossbar_l), )

storage = simulate!(mech, 1.0, controller!,
    record=true,
    verbose=true);

visualize(mech, storage, vis=vis);


z = get_maximal_state(mech)
set_robot(vis, mech, z)

angle(get_body(mech, :pair01_leg1_bar_k).state.q2)
storage = simulate!(mech, 10.0, controller!, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-1, btol=1e-1, undercut=5.0, verbose=true))
delete!(vis)
visualize(mech, storage, vis=vis, show_contact=false, build=true)

for b in mech.bodies
    println(b.name)
end


# Check joint limits
# for joint in mech.joints
#     print(joint.rotational.joint_limits)
# end
ddelete!(vis)
# vis = Visualizer()

set_minimal_coordinates!(mech, get_joint(mech, :pair01_joint_crank_axle_m), [0.0,0.0,4.0, 0,0,0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
out = get_joint(mech, :pair01_joint_crank_axle_m).rotational


models = []
normal = [0.0; 0.0; 1.0]
foot_radius = 0.0203

o = foot_radius

for body in mech.bodies
    if occursin("bars_g_h_i", string(body.name))
        println(body.name)

        push!(models, contact_constraint(body, normal;
        friction_coefficient,
        contact_origin=body.state.x1,
        contact_radius=o,
        contact_type,
        name=body.name))
    end
end
set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0,0,2.0, 0,0,0])
mech = Mechanism(Origin{T}(), mech.bodies, mech.joints, [models...];
            gravity,
            timestep)
z = get_maximal_state(mech)
z[6] = 5.0
# z
set_maximal_state!(mech, z)
# z = get_maximal_state(mech)
# z
# delete!(vis)
build_robot(mech, vis=vis)
set_robot(vis, mech, z, show_joint=true, show_contact=true)
## Hand done
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_joint_crank_axle_m); xmin=[-pi/2, 0])
z = get_maximal_state(mech)
delete!(vis)
build_robot(mech, vis=vis)
set_robot(vis, mech, z, show_joint=true, show_contact=true)

set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_m_j); xmin=[0, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
get_body(mech, get_joint(mech, :pair01_leg1_joint_m_j).parent_id).state.x2
motor = get_joint(mech, :pair01_leg1_joint_m_j).translational



set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_e); xmin=[pi/4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_e_f); xmin=[pi/4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)

set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_k); xmin=[pi/4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_c); xmin=[-pi/4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_i); xmin=[-pi/4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
storage = simulate!(mech, 0.4, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-2, btol=1e-2, undercut=5.0, verbose=true))
visualize(mech, storage, vis=vis, show_contact=true, build=true)
mech.joints

a = pi/2
for j in mech.joints
    if occursin("loop", string(j.name)) || occursin("floating", string(j.name))
        continue
    end
    println(j.name)
    # set_minimal_coordinates_velocities!(mech, j; xmin=[-a, a])
end

z = get_maximal_state(mech)
set_robot(vis, mech, z)


storage = simulate!(mech, 0.4, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-2, btol=1e-2, undercut=5.0, verbose=true))
visualize(mech, storage, vis=vis, show_contact=true, build=true)

out = get_minimal_coordinates(mech)
size(mech.joints)
set_minimal_coordinates!(mech, out)
for j in mech.joints
    println(j.name)
end

get_joint(mech, :floating_base)

a = 0
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_joint_crank_axle_m);
    xmin=[a, a])

z = get_maximal_state(mech)
set_robot(vis, mech, z)

for joint in mech.joints
    println(joint.name, " ", get_body(mech, joint.parent_id).name, " ", get_body(mech, joint.child_id).name," ", get_body(mech, joint.parent_id).state.x2, " ", get_body(mech, joint.child_id).state.x2)

end
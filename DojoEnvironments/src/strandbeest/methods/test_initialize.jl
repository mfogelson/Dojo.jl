# Variables
using Dojo
timestep=0.05
gravity=[0.0; 0.0; 0.0]
friction_coefficient=1.0
contact_foot=false
contact_body=false
limits=false
model=:strandbeest
floating=false
contact_type=:linear #non-linear
spring=0.0
damper=1.0
parse_damper=true
T=Float64
global REG = 1.0e-10::Float64

# Model 
path = joinpath(@__DIR__, "../deps/Strandbeest.urdf")
mech = Mechanism(path; floating, T,
gravity,
timestep,
parse_damper)
vis = Visualizer()
build_robot(mech, vis=vis)
# Adding springs and dampers
using DojoEnvironments
DojoEnvironments.set_springs!(mech.joints, spring)
DojoEnvironments.set_dampers!(mech.joints, damper)

set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint_crossbar_crank); xmin=[0, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# Bar J
# 5.07361093803733
theta = 5.07361093803733
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_m_j); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_m_j); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_m_j); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_m_j); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_m_j); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_m_j); xmin=[theta, 0])

# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_m_j); xmin=[theta-2pi, 0])

z = get_maximal_state(mech)
set_robot(vis, mech, z)
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_m_j); xmin=[-5.07361093803733, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_m_j); xmin=[-5.07361093803733, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_m_j); xmin=[-5.07361093803733, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_m_j); xmin=[-5.07361093803733, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_m_j); xmin=[-5.07361093803733, 0])


# Bar B
# 4.368635601032737
# Bar E
# 3.568982565099022
theta = -1.4
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_e); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_e); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_j_e); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_j_e); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_j_e); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_j_e); xmin=[theta, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_j_e); xmin=[theta-2pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_j_e); xmin=[1.4, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_j_e); xmin=[1.4, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_j_e); xmin=[1.4, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_j_e); xmin=[1.4, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_j_e); xmin=[1.4, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# Bar K
# 3.9677042625400483, -5.07361093803733
theta = 3.9677042625400483
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_k); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_k); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_j_k); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_j_k); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_j_k); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_j_k); xmin=[theta, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_j_k); xmin=[theta-2pi+pi/4, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_j_k); xmin=[5.07361093803733+pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_j_k); xmin=[5.07361093803733+pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_j_k); xmin=[5.07361093803733+pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_j_k); xmin=[5.07361093803733+pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_j_k); xmin=[5.07361093803733+pi, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# Bar C
# 1.8557540393718421
theta = -4.
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_c); xmin=[theta, 0])#997346692961635, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_c); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_k_c); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_k_c); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_k_c); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_k_c); xmin=[theta, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_k_c); xmin=[theta-2pi, 0])#997346692961635, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_k_c); xmin=[4.7, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_k_c); xmin=[4.7, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_k_c); xmin=[4.7, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_k_c); xmin=[4.7, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_k_c); xmin=[4.7, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# Bar F
# 5.118452698935833
theta = -1.
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_e_f); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_e_f); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_e_f); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_e_f); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_e_f); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_e_f); xmin=[theta, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_e_f); xmin=[theta-2pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_e_f); xmin=[1.8557540393718421, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_e_f); xmin=[1.8557540393718421, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_e_f); xmin=[1.8557540393718421, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_e_f); xmin=[1.8557540393718421, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_e_f); xmin=[1.8557540393718421, 0])
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# Bar I 
theta = -0.5
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_i); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_i); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg1_joint_k_i); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg1_joint_k_i); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg1_joint_k_i); xmin=[theta, 0])
set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg1_joint_k_i); xmin=[theta, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg2_joint_k_i); xmin=[theta-2pi, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg2_joint_k_i); xmin=[0.5, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair03_leg2_joint_k_i); xmin=[0.5, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair04_leg2_joint_k_i); xmin=[0.5, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair05_leg2_joint_k_i); xmin=[0.5, 0])
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair06_leg2_joint_k_i); xmin=[0.5, 0])
# Check joint limits
# for joint in mech.joints
#     print(joint.rotational.joint_limits)
# end
z = get_maximal_state(mech)
set_robot(vis, mech, z)
# vis = Visualizer()
build_robot(mech, vis=vis)
z = get_maximal_state(mech)
set_robot(vis, mech, z)

# set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0.0,0.0,3.0, 0,0,0])

function controller!(m, t)
    set_input!(get_joint(m, :joint_crossbar_crank), 10.0*SVector(rand()))
    # x = get_minimal_state(mech)
    # x_goal = fill!(similar(x), 0)
    # # x_goal[7] = 0.0
    # K = fill!(similar(x), 0.1)
    # u =@. -K * (x - x_goal)
    # set_input!(mech, u)
    return nothing
end

storage = simulate!(mech, 10.0, controller!, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-2, btol=1e-2, undercut=5.0, verbose=true))
# delete!(vis)
visualize(mech, storage, vis=vis, show_contact=false, build=true, show_joint=false)


for x in get_minimal_state(mech)
    println(x)
end

set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0.0,0.0,2.0, 0,0,0])

storage = simulate!(mech, 2.0, controller!,
    record=true,
    verbose=true);

visualize(mech, storage, vis=vis)


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
mech = Mechanism(Origin{T}(), mech.bodies, mech.joints;
            gravity,
            timestep)
z = get_maximal_state(mech)
z[6] = 5.0
# z
set_maximal_state!(mech, z)
# z = get_maximal_state(mech)
# z
delete!(vis)
build_robot(mech, vis=vis)
set_robot(vis, mech, z, show_joint=false, show_contact=true)

storage = simulate!(mech, 1.0, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-4, btol=1e-4, undercut=5.0, verbose=true))
visualize(mech, storage, vis=vis, show_contact=true, build=true)

out = get_minimal_coordinates(mech)
size(mech.joints)
set_minimal_coordinates!(mech, out)

# Set minimal coordinates recursively 

## Test simulation

# z = get_maximal_state(mech)
# vis=Visualizer()
# build_robot(mech, vis=vis, show_joint=false, show_contact=true)


# # get_joint(mech, :floating_base).translational
# # get_joint(mech, :floating_base).rotational

# # get_minimal_state(mech)
# # # Visualize
# # vis=Visualizer()
# # 

# # joint limit
# # Contact

# get_joint(mech, :floating_base)

# get_body(mech, :crossbar)
# build_robot(mech, vis=vis)
# get_joint(mech, 185)

# for joint in mech.joints
#     !(joint.name in (:loop_joint, :floating_joint)) && set_minimal_coordinates!(mech, joint, zeros(input_dimension(joint)))
# end
# set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0,0,1.0, 0,0,0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=true, show_contact=true)

# Variables
using Dojo
timestep=0.01
gravity=[0.0; 0.0; -9.81]
friction_coefficient=1.0
contact_foot=true
contact_body=true
limits=true
model=:strandbeest
floating=false
contact_type=:nonlinear
spring=0.0
damper=0.0
parse_damper=true
T=Float64

# Model 
path = joinpath(@__DIR__, "../deps/Strandbeest.urdf")
mech = Mechanism(path; floating, T,
gravity,
timestep,
parse_dampers=parse_damper)

initialize_constraints!(mech)

vis = Visualizer()
delete!(vis)
Dojo.build_robot(mech, vis=vis, visualize_floor=false)
Dojo.set_robot(vis, mech, get_maximal_state(mech))
z = get_maximal_state(mech)
Dojo.initialize_state!(mech)
z_rand = z .+ rand(length(z))
set_maximal_state!(mech, z_rand)
zero_velocities!(mech)
z_new = get_maximal_state(mech)
Dojo.set_entries!(mech) # compute the residual
initialize_constraints!(mech, debug=true, regularization=1e-10, fixedids=[185])
delete!(vis)
visualize(mech, vis=vis, visualize_floor=false)

ids = [body.id for body in mech.bodies]
dist = [Dojo.norm(body.state.x2) for body in mech.bodies]
argmax(dist)
for body in mech.bodies
    body.state.x2 = body.state.x2 .- mech.bodies[argmax(dist)].state.x2
    body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
end 

length(mech.joints)
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

storage = simulate!(mech, 1.0, controller!, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1e-2, btol=1e-2, undercut=5.0, verbose=true))

# Adding springs and dampers
using DojoEnvironments
DojoEnvironments.set_springs!(mech.joints, spring)
DojoEnvironments.set_dampers!(mech.joints, damper)

# Check joint limits
# for joint in mech.joints
#     print(joint.rotational.joint_limits)
# end

set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0.0,0.0,3.0, 0,0,0])

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
set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0,0,1.0, 0,0,0])
mech = Mechanism(Origin{T}(), mech.bodies, mech.joints, [models...];
            gravity,
            timestep)
# z = get_maximal_state(mech)
# z[3] = 2.0
# z
# set_maximal_state!(mech, z)
# z = get_maximal_state(mech)
# z
build_robot(mech, vis=vis)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
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

# Variables
using Dojo
timestep = 0.001
gravity = [0.0; 0.0; -9.81]
friction_coefficient = 1.0
contact_foot = false
contact_body = false
limits = false
model = :strandbeest
floating = true
contact_type = :linear #non-linear
spring = 1.0 # P values (only for crank_axle joint)
damper = 10.0 # D values for target input
parse_damper = true
T = Float64
global REG = 1.0e-4::Float64
vis = Visualizer()
setprop!(vis["/Background"], "top_color", colorant"transparent")

## Visualize
function add_axes!(vis,axes_name, scale, R; head_l = 0.05, head_w = 0.02, r = zeros(3), q = UnitQuaternion())
    red_col =RGBA(1.0,0.0,0.0,0.5)
    green_col =RGBA(0.0,1.0,0.0,0.5)
    blue_col =RGBA(0.0,0.0,1.0,0.5)

    cylx = GeometryBasics.Cylinder(Point(0,0,0.0), Point(scale,0,0.0), R)
    cyly = GeometryBasics.Cylinder(Point(0,0,0.0), Point(0,scale,0.0), R)
    cylz = GeometryBasics.Cylinder(Point(0,0,0.0), Point(0,0.0,scale), R)

    setobject!(vis[axes_name][:x], cylx, MeshPhongMaterial(color=red_col))
    head = Cone(Point(scale,0,0.0), Point(scale + head_l, 0.0, 0), head_w)
    setobject!(vis[axes_name][:hx], head, MeshPhongMaterial(color=red_col))

    setobject!(vis[axes_name][:y], cyly, MeshPhongMaterial(color=green_col))
    head = Cone(Point(0,scale,0.0), Point(0, scale + head_l, 0.0), head_w)
    setobject!(vis[axes_name][:hy], head, MeshPhongMaterial(color=green_col))

    setobject!(vis[axes_name][:z], cylz, MeshPhongMaterial(color=blue_col))
    head = Cone(Point(0,0.0,scale), Point(0, 0.0, scale + head_l), head_w)
    setobject!(vis[axes_name][:hz], head, MeshPhongMaterial(color=blue_col))

    settransform!(vis[axes_name],Translation(r) ∘ LinearMap(q))

    return nothing
end

function set_robot(vis::Visualizer, mechanism::Mechanism, z::Vector{T};
    show_joint::Bool=false,
    joint_radius=0.1,
    show_contact::Bool=true, 
    show_tf::Bool=true,
    name::Symbol=:robot) where {T,N}

    (length(z) == minimal_dimension(mechanism)) && (z = minimal_to_maximal(mechanism, z))
    bodies = mechanism.bodies
    origin = mechanism.origin

    # Bodies and Contacts
    for (id, body) in enumerate(bodies)
        x, _, q, _ = unpack_maximal_state(z, id)
        shape = body.shape
        visshape = convert_shape(shape)
        subvisshape = nothing
        showshape = false
        if visshape !== nothing
            subvisshape = vis[name][:bodies][Symbol(body.name, "__id_$id")]
            showshape = true
        end

        set_node!(x, q, id, shape, subvisshape, showshape)
        
        if show_joint
            for (jd, joint) in enumerate(mechanism.joints)
                # add_axes!(vis, joint.name, 0.05, 0.01, r=x, q=q)

                if joint.child_id == body.id
                    # radius = joint_radius
                    # joint_shape = Sphere(radius,
                    #     position_offset=joint.translational.vertices[2],
                    #     color=RGBA(0.0, 0.0, 1.0, 0.5))
                    # # joint_shape = Triad(x, q)
                    # visshape = convert_shape(joint_shape)
                    # subvisshape = nothing
                    # showshape = false
                    # if visshape !== nothing
                    #     subvisshape = vis[name][:joints][Symbol(joint.name, "__id_$(jd)")]
                    #     showshape = true
                    # end
                    # set_node!(x, q, id, joint_shape, subvisshape, showshape)
                    add_axes!(vis, joint.name, 0.05, 0.01, r=x, q=q)
                end
            end
        end
        if show_contact
            for (jd, contact) in enumerate(mechanism.contacts)
                if contact.parent_id == body.id
                    radius = abs(contact.model.collision.contact_radius)
                    (radius == 0.0) && (radius = 0.01)
                    contact_shape = Sphere(radius,
                        position_offset=(contact.model.collision.contact_origin),
                        orientation_offset=one(Quaternion), color=RGBA(1.0, 0.0, 0.0, 0.5))
                    visshape = convert_shape(contact_shape)
                    subvisshape = nothing
                    showshape = false
                    if visshape !== nothing
                        subvisshape = vis[name][:contacts][Symbol(contact.name, "__id_$(jd)")]
                        showshape = true
                    end
                    set_node!(x, q, id, contact_shape, subvisshape, showshape)
                end
            end
        end


        
    end

    # Origin
    id = origin.id
    shape = origin.shape
    visshape = convert_shape(shape)
    if visshape !== nothing
        subvisshape = vis[name][:bodies][Symbol(:origin, "_id")]
        shapetransform = transform(szeros(T,3), one(Quaternion{T}), shape)
        settransform!(subvisshape, shapetransform)
    end
    return vis
end

# Load Model 
path = joinpath(@__DIR__, "../deps/Strandbeest.urdf")
mech = Mechanism(path; floating, T,
    gravity,
    timestep,
    parse_damper)

# Adding springs and dampers
using DojoEnvironments
DojoEnvironments.set_springs!(mech.joints, spring)
DojoEnvironments.set_dampers!(mech.joints, damper)

## Add Contacts
models = []
normal = [0.0; 0.0; 1.0]
foot_radius = 0.0203

o = foot_radius

for body in mech.bodies
    if occursin("bars_g_h_i", string(body.name))
        println(body.name)

        push!(models, contact_constraint(body, normal;
            friction_coefficient,
            contact_origin=[0.0, 0.0, 0.490],
            contact_radius=o,
            contact_type,
            name=body.name))
    end
    # if occursin("crossbar", string(body.name))
    #     println(body.name)

    #     push!(models, contact_constraint(body, normal;
    #         friction_coefficient,
    #         contact_origin=[0.0, 0.0, 0.0],
    #         contact_radius=o,
    #         contact_type,
    #         name=body.name))
    # end
end
# floating = true
# gravity = [0.0, 0.0, -9.81]
mech = Mechanism(Origin{T}(), mech.bodies, mech.joints, [models...];
    gravity,
    timestep)
get_minimal_coordinates(mech)
residual_violation(mech)
bilinear_violation(mech)

using Optim
function solve_initial_configuration!(mech)

    # get joint limits:
    lb = similar(collect(values(get_minimal_coordinates(mech)))
    ) 
    ub = similar(collect(values(get_minimal_coordinates(mech)))
    )
    # for joint in mech.joints
    #     lb_joint = lb[joint]
    #     ub_joint = ub[joint]
    #     if isfloating(joint)
    #         # use identity transform for floating joint
    #         tf = one(Transform3D{T}, frame_after(joint), frame_before(joint))
    #         set_configuration!(state, joint, tf)
    #         # Optim can't handle equal bounds, so give it a bit of wiggle room
    #         lb_joint .= configuration(state, joint) .- 1e-3
    #         ub_joint .= configuration(state, joint) .+ 1e-3
    #     else
    #         lb_joint .= rbd.lower.(position_bounds(joint))
    #         ub_joint .= rbd.upper.(position_bounds(joint))
    #     end
    # end

    ## Set all joints to midpoint
    # q = configuration(state)
    # for i in eachindex(q)
    #     if isfinite(lb[i]) && isfinite(ub[i])
    #         q[i] = (lb[i] + ub[i]) / 2
    #     else
    #         q[i] = clamp(0, lb[i], ub[i])
    #     end
    # end
    for i in 3:4
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_joint_crank_axle_m")).rotational.joint_limits
        println()
        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_joint_crank_axle_m")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
    
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_m_j")); xmin=[5.07361093803733, 0])
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_m_j")).rotational.joint_limits
        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_m_j")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_m_j); xmin=[5.07361093803733 - 2pi, 0])
    
        # Bar B
        # 4.368635601032737
        # Bar E
        # 3.568982565099022
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_e")).rotational.joint_limits
        # print(((lb+ub)/2.0)[1])
        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_e")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_e); xmin=[-1.4 - 2pi, 0])
    
        # Bar K
        # 3.9677042625400483
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_k")).rotational.joint_limits

        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_k")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_k); xmin=[-4.5 - pi - 2pi, 0])
    
        # Bar C
        # 1.8557540393718421
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_c")).rotational.joint_limits

        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_c")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_c); xmin=[-4.7 - 2pi, 0])#997346692961635, 0])
    
        # Bar F
        # 5.118452698935833
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_e_f")).rotational.joint_limits

        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_e_f")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_e_f); xmin=[-1 - 2pi, 0])
    
        # Bar I 
        lb, ub = get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_i")).rotational.joint_limits

        set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_i")); xmin=[isempty((lb+ub)/2.0) ? 0 : (lb+ub)[1]/2.0, 0])
        # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_i); xmin=[-0.5 - 2pi, 0])
    end
    ## not sure
    # setdirty!(state)

    # Use Optim's Fminbox to minimize the loop joint error within
    # the joint limits:
    cost = residual_violation(mech)
    result = Optim.optimize(cost, lb, ub,
        collect(values(get_minimal_coordinates(mech))),
        Fminbox(Optim.LBFGS()), autodiff=:forward)

    # Verify that we've actually closed all the loops
    @assert Optim.minimum(result) < 1e-9

    set_minimal_coordinates!(mech, Optim.minimizer(result))

    # set_configuration!(state, Optim.minimizer(result))
    # normalize_configuration!(state)
    state
end
# collect(values(get_minimal_coordinates(mech)))
solve_initial_configuration!(mech)

## Load IC 
z = load_object("maximal_state_biped_normal.jld2") #"maximal_close_biped.jld2"
set_maximal_state!(mech, z)

## Update Vis
delete!(vis)
build_robot(mech, vis=vis, show_contact=true)
z = get_maximal_state(mech)
set_robot(vis, mech, z, show_joint=false, show_contact=true)
# get_body(mech, :crossbar).state
# set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0, 0, 0.83, 0, 0, 0.0])
# save_object("maximal_state_biped_normal.jld2", z)
z
function controller!(m, t)
    set_input!(get_joint(m, :joint_crossbar_crank), 0.0*SVector(rand()))
    return nothing
end

storage = simulate!(mech, 300 * timestep, controller!, record=true, abort_upon_failure=false,
    opts=SolverOptions(rtol=1.0e-4,
                        btol=1.0e-2,
                        ls_scale=0.5,
                        max_iter=100,
                        max_ls=10,
                        undercut=Inf,
                        no_progress_max=3,
                        no_progress_undercut=10.0,
                        verbose=true));
visualize(mech, storage, vis=vis, show_contact=true, build=true, show_joint=false);


offset = 2.0
for i in 3:4
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_joint_crank_axle_m")); xmin=[0, 0])

    # set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_m_j")); xmin=[5.07361093803733, 0])
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_m_j")); xmin=[i%3==0 ? 2.7 : 5.07361093803733, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_m_j); xmin=[5.07361093803733 - 2pi, 0])

    # Bar B
    # 4.368635601032737
    # Bar E
    # 3.568982565099022
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_e")); xmin=[-1.4, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_e); xmin=[-1.4 - 2pi, 0])

    # Bar K
    # 3.9677042625400483
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_j_k")); xmin=[-1.36, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_j_k); xmin=[-4.5 - pi - 2pi, 0])

    # Bar C
    # 1.8557540393718421
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_c")); xmin=[-3.6, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_c); xmin=[-4.7 - 2pi, 0])#997346692961635, 0])

    # Bar F
    # 5.118452698935833
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_e_f")); xmin=[-1.0, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_e_f); xmin=[-1 - 2pi, 0])

    # Bar I 
    set_minimal_coordinates_velocities!(mech, get_joint(mech, Symbol("pair0$i" * "_leg1_joint_k_i")); xmin=[-0.5, 0])
    # set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair02_leg1_joint_k_i); xmin=[-0.5 - 2pi, 0])
end
# set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0, 0, 1.2, 0, 0, 0.3])



# set_minimal_coordinates_velocities!(mech, get_joint(mech, :joint_crossbar_crank); xmin=[pi/2, 0])
# save_object("maximal_state_biped.jld2", z)
# # ## Add Contact to foot
# models = []
# normal = [0.0; 0.0; 1.0]
# foot_radius = 0.0203

# o = foot_radius

# for body in mech.bodies
#     if occursin("bars_g_h_i", string(body.name))
#         println(body.name)

#         push!(models, contact_constraint(body, normal;
#             friction_coefficient,
#             contact_origin=[0.0, 0.0, 0.490],
#             contact_radius=o,
#             contact_type,
#             name=body.name))
#     end
# end
# floating = true
# gravity = [0.0, 0.0, -9.81]
# mech = Mechanism(Origin{T}(), mech.bodies, mech.joints, [models...];
#     gravity,
#     timestep)


# delete!(vis)
# build_robot(mech, vis=vis, show_joint=false, show_contact=true)
# z = get_maximal_state(mech)
# set_robot(vis, mech, z)
# get_joint(mech, :floating_base)
# set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0, 0, 4.8, 0, 0, 0])
# minimum(z)
# # set_input!(mech, 100.0 * SVector(0.0, 0.0, -rand(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
# # z = get_maximal_state(mech)
# # set_robot(vis, mech, z)

# # for x in mech.joints
# #     println(x.name)
# # end
# # # get_minimal_state(mech)
# # for (key, val) in get_minimal_coordinates(mech)
# #     println(key)
# #     println(get_joint(mech, key).name)
# #     # println(get_joint(mech, key).translational)

# # end
# # using Flatten
# # fill!(collect(values(get_minimal_coordinates(mech))), [0.1])
# # for z in 
# #     println(z)
# # end
# # # z = get_minimal_coordinates(mech)
# # # z[13]
# # # z[8] = [1.57]
# # # set_minimal_coordinates!(mech, z)
get_joint(mech, 7).rotational.joint_limits

# # set_minimal_coordinates!(mech, get_joint(mech, :pair01_joint_crossbar_l), )

# # storage = simulate!(mech, 1.0, controller!,
# #     record=true,
# #     verbose=true);

# # visualize(mech, storage, vis=vis);


# # z = get_maximal_state(mech)
# # set_robot(vis, mech, z)

# # angle(get_body(mech, :pair01_leg1_bar_k).state.q2)
# # storage = simulate!(mech, 10.0, controller!, record=true, abort_upon_failure=false)
# # storage = simulate!(mech, 10.0, controller!, record=true, abort_upon_failure=false,
# #     opts=SolverOptions(rtol=1e-4, btol=1e-4, undercut=5.0, verbose=true))
# # delete!(vis)
# # visualize(mech, storage, vis=vis, show_contact=false, build=true)

# # for b in mech.bodies
# #     println(b.name)
# # end


# # Check joint limits
# # for joint in mech.joints
# #     print(joint.rotational.joint_limits)
# # end
# # delete!(vis)
# # # vis = Visualizer()

# # set_minimal_coordinates!(mech, get_joint(mech, :pair01_joint_crank_axle_m), [0.0, 0.0, 4.0, 0, 0, 0])
# # z = get_maximal_state(mech)
# # set_robot(vis, mech, z)
# # out = get_joint(mech, :pair01_joint_crank_axle_m).rotational



# # set_minimal_coordinates!(mech, get_joint(mech, :floating_base), [0, 0, 2.0, 0, 0, 0])
# delete!(vis)

# z = get_maximal_state(mech)

# build_robot(mech, vis=vis, show_contact=true)
# set_robot(vis, mech, z, show_joint=true, show_contact=true)
# ## Hand done
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_joint_crank_axle_m); xmin=[-pi / 2, 0])
# z = get_maximal_state(mech)
# delete!(vis)
# build_robot(mech, vis=vis)
# set_robot(vis, mech, z, show_joint=true, show_contact=true)

# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_m_j); xmin=[0, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)
# get_body(mech, get_joint(mech, :pair01_leg1_joint_m_j).parent_id).state.x2
# motor = get_joint(mech, :pair01_leg1_joint_m_j).translational



# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_e); xmin=[pi / 4, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_e_f); xmin=[pi / 4, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)

# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_j_k); xmin=[pi / 4, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_c); xmin=[-pi / 4, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_leg1_joint_k_i); xmin=[-pi / 4, 0])
# z = get_maximal_state(mech)
# set_robot(vis, mech, z, show_joint=false, show_contact=true)
# storage = simulate!(mech, 0.4, record=true, abort_upon_failure=false,
#     opts=SolverOptions(rtol=1e-2, btol=1e-2, undercut=5.0, verbose=true))
# visualize(mech, storage, vis=vis, show_contact=true, build=true)
# mech.joints

# a = pi / 2
# for j in mech.joints
#     if occursin("loop", string(j.name)) || occursin("floating", string(j.name))
#         continue
#     end
#     println(j.name)
#     # set_minimal_coordinates_velocities!(mech, j; xmin=[-a, a])
# end

# z = get_maximal_state(mech)
# set_robot(vis, mech, z)

# for b in mech.bodies
#     b.mass = 0.0
#     # b.inertia = I(3)
#     # println(b.inertia)
# end
# storage = simulate!(mech, 0.4, record=true, abort_upon_failure=false,
#     opts=SolverOptions(rtol=1e-4, btol=1e-4, undercut=5.0, verbose=true))
# visualize(mech, storage, vis=vis, show_contact=true, build=true)

# out = get_minimal_coordinates(mech)
# size(mech.joints)
# set_minimal_coordinates!(mech, out)
# for j in mech.joints
#     println(j.name)
# end

# get_joint(mech, :floating_base)

# a = 0
# set_minimal_coordinates_velocities!(mech, get_joint(mech, :pair01_joint_crank_axle_m);
#     xmin=[a, a])

# z = get_maximal_state(mech)
# set_robot(vis, mech, z)

# for joint in mech.joints
#     println(joint.name, " ", get_body(mech, joint.parent_id).name, " ", get_body(mech, joint.child_id).name, " ", get_body(mech, joint.parent_id).state.x2, " ", get_body(mech, joint.child_id).state.x2)

# end
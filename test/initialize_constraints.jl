using Dojo
using FiniteDiff
include("make_scissor.jl")

# Define the mechanism
mechanism = initialize_mechanism()
vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false)
z = get_maximal_state(mechanism)
# z = randn(length(z))
# set_maximal_state!(mechanism, z)

# for body in mechanism.bodies
#     body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
#     body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
#     println(Dojo.norm(body.state.q2))
# end
# Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2

function wrapper_func(mechanism, z)
    set_maximal_state!(mechanism, z)
    for body in mechanism.bodies
        body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
        # println(Dojo.norm(body.state.q2))
    end
    Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2
    res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
    return 0.5*res'*res
end

using FiniteDiff
# FD_jac = FiniteDiff.finite_difference_gradient(_z  -> wrapper_func(mechanism, _z), z)
# FD_hes = FiniteDiff.finite_difference_hessian(_z  -> wrapper_func(mechanism, _z), z)
# con_jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, mechanism.bodies, false)
# update_constraint_jacobian!(con_jac, mechanism, mechanism.bodies, joint_idx, body_idx, false)

# FD_jac[1:3, 7:10] ./ con_jac[1:3, 4:7]
# con_jac[1:3, 4:7] ./ FD_jac[1:3, 7:10]

function initialize_constraint_jacobian(mechanism::Mechanism, freeids::Set{Int64}, attjac=true)
    # Getting the degrees of freedom for each joint
    joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]

    body_dof = attjac ? 6 : 7
    # Creating index ranges for the joints
    joint_idx = Dojo.index_ranges(joint_dof)

    # Calculating total number of constraints and bodies
    num_constraints = sum(joint_dof)
    num_bodies = body_dof*length(freeids)

    # Creating index ranges for the bodies
    body_idx = Dojo.index_ranges([body_dof for _ in freeids])

    # Creating the constraint matrix
    con_jac =　zeros(Float64, num_constraints, num_bodies)

    return con_jac, joint_idx, body_idx
end

function update_constraint_jacobian!(con_jac::AbstractMatrix, mechanism::Mechanism, freebodies::Vector{Body{Float64}}, joint_idx, body_idx, attjac=true)
    # Filling the constraint Jacobian
    for (i,(joint, j_idx)) in enumerate(zip(mechanism.joints, joint_idx))
        for (j,(body, b_idx)) in enumerate(zip(freebodies, body_idx))
            if body.id == joint.parent_id
                con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body, attjac)
            elseif body.id == joint.child_id
                con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body, attjac)
            end
        end
    end
end

function get_child_body(mechanism, joint)
    return Dojo.get_body(mechanism, joint.child_id)
end

function get_parent_body(mechanism, joint)
    return Dojo.get_body(mechanism, joint.parent_id)
end


# TODO: Test why finite diff does not match constraint_jacobian_configuration
# function wrapped_constraint_function(_x)
#     result = Dojo.constraint(joint1.translational, link1.state.x2, Quaternion(_x...), link2.state.x2, link2.state.q2, joint1.impulses[2][Dojo.joint_impulse_index(joint1,1)], mechanism.μ)
#     return SVector(result...)  # Ensure result is an SVector if required
# end

# using FiniteDiff
# FiniteDiff.finite_difference_jacobian(_x -> Dojo.constraint(joint1.translational, _x, link1.state.q2, link2.state.x2, link2.state.q2, joint1.impulses[2][Dojo.joint_impulse_index(joint1,1)], mechanism.μ), link1.state.x2)
# FiniteDiff.finite_difference_jacobian(_x -> Dojo.constraint(joint1.rotational, _x, link1.state.q2, link2.state.x2, link2.state.q2, joint1.impulses[2][Dojo.joint_impulse_index(joint1,1)], mechanism.μ), link1.state.x2)
# FiniteDiff.finite_difference_jacobian(_x -> wrapped_constraint_function(_x), [link1.state.q2.s, link1.state.q2.v1, link1.state.q2.v2, link1.state.q2.v3])

# FiniteDiff.finite_difference_jacobian(_x -> Dojo.constraint(joint1.translational, link1.state.x2, link1.state.q2, _x, link2.state.q2, joint1.impulses[2][Dojo.joint_impulse_index(joint1,1)], mechanism.μ), link2.state.x2)
# FiniteDiff.finite_difference_jacobian(_x -> Dojo.constraint(joint1.translational, link1.state.x2, link1.state.q2, link2.state.x2, _x, joint1.impulses[2][Dojo.joint_impulse_index(joint1,1)], mechanism.μ), link2.state.q2)
# TODO check why finite diff does not match constraint_jacobian_configuration
using LinearAlgebra
using SparseArrays

function pack_maximal_configuration!(z::AbstractVector, 
	x2::AbstractVector, q2::Quaternion, i::Int)
    start_idx = (i-1)*7+1
	z[Dojo.SUnitRange(start_idx, start_idx+2)] .= x2
	z[Dojo.SUnitRange(start_idx+3, start_idx+6)] .= Dojo.vector(q2)

	return nothing
end

function get_maximal_configuration(mechanism::Mechanism{T,Nn,Ne,Nb,Ni}, freeids::Set{Int64}) where {T,Nn,Ne,Nb,Ni}
    z = zeros(T, 7*length(freeids))
    i = 0
    for body in mechanism.bodies
        if body.id in freeids
            i += 1
            pack_maximal_configuration!(z, body.state.x2, body.state.q2, i)
        end
    end
    return z
end

function unpack_step(data::AbstractVector)
    x2 = data[Dojo.SA[1;2;3]]
    q2 = data[Dojo.SA[4;5;6]]
    return x2, q2
end

function unpack_configuration(data::AbstractVector)
    x2 = data[Dojo.SA[1;2;3]]
    q2 = data[Dojo.SA[4;5;6;7]]
    return x2, q2
end

function set_configuration!(mechanism::Mechanism, freeids::Set{Int64}, z::AbstractVector)
    off = 0
    for body in mechanism.bodies
        if body.id in freeids
            x, q = unpack_configuration(z[Dojo.SUnitRange(off+1,end)]); off += 7
            q = Quaternion(normalize(q)...)
            body.state.x1 = x
            body.state.q1 = q

            body.state.x2 = x
            body.state.q2 = q
        end
    end
end

function update_stepvec!(stepvec::AbstractVector, res::AbstractVector, jac::AbstractMatrix; debug=false)

    F = svd(jac, full=true, alg=LinearAlgebra.QRIteration())
    new_rank = sum(F.S .> 1e-6)
    if debug
        println("rank: $new_rank | full: $(size(F.V, 2))")
    end


    V1 = @view F.V[:,1:new_rank]
    S1 = @view F.S[1:new_rank]
    U1 = @view F.U[:,1:new_rank]

    stepvec[:] .= -V1*Diagonal(1.0 ./ S1)*U1'*res
end

# function update_stepvec!(stepvec::AbstractVector, grad::AbstractVector, hes::AbstractMatrix, mechanism::Mechanism)
#     Dojo.rank(hes)

#     F = svd(hes, full=true, alg=LinearAlgebra.QRIteration())
#     new_rank = sum(F.S .> 1e-6)
#     println("rank: $new_rank | full: $(size(F.V, 2))")

#     V1 = @view F.V[:,1:new_rank]
#     S1 = @view F.S[1:new_rank]
#     U1 = @view F.U[:,1:new_rank]

#     stepvec[:] .= -V1*Diagonal(1.0 ./ S1)*U1'*grad
# end

# 0.5*(b+con_jac*stepvec)'*(b+con_jac*stepvec)
# b =  Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))

function get_residual(mechanism::Mechanism, freeids::Set{Int64}, z::AbstractVector{Float64})
    set_configuration!(mechanism, freeids, z)
    
    return Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
end

# Function to compute the objective function (user-defined)
function objective_function(mechanism::Mechanism, freebodies::Vector{Body{Float64}}, freeids::Set{Int64}, jac::AbstractMatrix{Float64}, joint_idx, body_idx,  z::AbstractVector{Float64}, dz::AbstractVector{Float64})
    # println(size(z), size(dz))
    res = get_residual(mechanism, freeids, z)
    # jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freebodies, true)
    update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx, true)
    # println(size(res), size(jac))

    return 0.5*(res+jac*dz)'*(res+jac*dz)
end

# Function to perform Armijo line search
function armijo_line_search(dz::AbstractVector{Float64}, mechanism::Mechanism, freebodies, freeids::Set{Int64}, jac, joint_idx, body_idx, α=1e-4, β=0.1, max_iter=100; debug=false)
    t = 1.0

    z = get_maximal_configuration(mechanism, freeids)
    # initial_obj = objective_function(mechanism, freebodies, z, dz.*t)
    
    initial_obj = norm(get_residual(mechanism, freeids, z))
    grad = FiniteDiff.finite_difference_gradient(_dz -> objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, _dz), dz)

    for i in 1:max_iter
        # Create a temporary mechanism to test the step
        temp_mechanism = deepcopy(mechanism)
        set_configuration!(temp_mechanism, freeids, z)
      
        newton_step(dz, temp_mechanism, freeids, t=t)
        
        z_new = get_maximal_configuration(temp_mechanism, freeids)
        new_obj = norm(get_residual(temp_mechanism, freeids, z_new)) #objective_function(temp_mechanism, freebodies, z_new, dz.*t)
        
        if debug
            println("New obj: $new_obj | Initial obj: $(initial_obj) | Update: $(α * t * dot(grad, dz)) | α: $α | β: $β | t: $t")
        end
        
        if new_obj <= initial_obj + α * t * dot(grad, dz)
            if debug
                println("Line search converged")
            end
 
            return t
        else
            t *= β
        end
    end
    if debug
        println("Line search did not converge")
    end
    return t  # Return the step size found
end

# Function to update the mechanism with a bounded angular step and Armijo line search
function newton_step(stepvec::AbstractVector, mechanism::Mechanism, freeids; t = 1.0, debug=false)

    # TODO Bound the angular step vector
    # max_angle = π / 4  # Example bound, adjust as necessary
    off = 0
    # freeids = Dojo.getid.(freebodies)
    for body in mechanism.bodies
        if !(body.id in freeids)
            continue
        end
        dx, dq = unpack_step(stepvec[Dojo.SUnitRange(off+1, off+6)])
        off += 6

        # scale update
        dx *= t
        dq *= t

        dq_norm = norm(dq)
        if dq_norm > 1.0
            dq *= (1.0 / dq_norm)
            if debug
                println(dq_norm)
            end
        end

        w = sqrt(1-min(1, norm(dq))^2)
        dq = [w, dq...]
        dq = normalize(dq)

        body.state.x2 = body.state.x1 .+ dx 

        body.state.q2 = body.state.q1 * Quaternion(dq...)
    end
end

# function attitude_jacobian(data::AbstractVector, Nb::Int)
#     G = zeros(0,0)
#     off = 0
#     for i = 1:Nb
#         x, q = unpack_configuration(data[Dojo.SUnitRange(off+1,end)]); off += 7
#         q = Quaternion(q...)
#         G = cat(G, Dojo.LVᵀmat(q), dims = (1,2))
#     end
#     ndata = length(data)
#     nu = ndata - size(G)[1]
#     G = cat(G, I(nu), dims = (1,2))
#     return G
# end

function get_free_bodies(mechanism; fixedids = Int64[], freeids = Int64[],)
    # Ensure mutual exclusivity of fixedids and freeids
    if !isempty(fixedids) && !isempty(freeids)
        error("Specify either free or fixed bodies, not both.")
    end

    # Determine freeids if only fixedids are specified
    if !isempty(fixedids)
        all_ids = Dojo.getid.(mechanism.bodies)
        freeids = filter(id -> !(id in fixedids), all_ids)
    elseif isempty(freeids)
        freeids = Dojo.getid.(mechanism.bodies)
    end

    # Initialize the array of free bodies
    freebodies = [get_body(mechanism, id) for id in freeids]

    return freebodies, Set(freeids)
end


function initialize_joint_constraints(mechanism::Mechanism{T}, z0::AbstractVector; fixedids = Int64[], freeids = Int64[], ε = 1e-5, newtonIter = 100, lineIter = 10, regularization = 1e-6, debug=false, vis=nothing) where T
    freebodies, freeids = get_free_bodies(mechanism, fixedids=fixedids, freeids=freeids)
    # set initial configuration 
    set_configuration!(mechanism, freeids, z0)

    # get the initial configuration
    z = get_maximal_configuration(mechanism, freeids)

    # Get the initial maximum violation of constraints
    # println(z0)
    n = length(z)
    m = length(freebodies)*6
    dz = zeros(Float64, m)

    jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freeids, true)

    norm0 = objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, dz) #maximum(violations(mechanism))



    # grad = zeros(Float64, n)
    # FiniteDiff.finite_difference_gradient!(grad, _z -> objective_function(mechanism, freebodies, _z), z)
    
    # hes = zeros(Float64, n, n)
    # FiniteDiff.finite_difference_hessian!(hes, _z -> objective_function(mechanism, freebodies, _z), z)

    # G = attitude_jacobian(z, length(freebodies))
    # grad_att = grad * G
    # hes_att = G'*hes*G - I*grad*
    zero_dz = zero(dz)
    # Newton-Raphson iterations
    for i in Base.OneTo(newtonIter)
        if debug
            println("Iter: $i | Norm: $norm0")
        end

        # Copy state variables from previous step
        for body in freebodies
            body.state.x1 = 1.0*body.state.x2
            body.state.q1 = 1.0*body.state.q2
        end
        z = get_maximal_configuration(mechanism, freeids)

        # Update the constraint Jacobian
        # update_constraint_jacobian!(con_jac, mechanism, freebodies, joint_idx, body_idx, true)
        # FiniteDiff.finite_difference_hessian!(hes, _z -> objective_function(mechanism, freebodies, _z), z)
        # FiniteDiff.finite_difference_gradient!(grad, _z -> objective_function(mechanism, freebodies, _z), z)
        res = get_residual(mechanism, freeids, z)

        update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx, true)
        # Update the step vector
        update_stepvec!(dz, res, jac)
    

        # Perform Armijo line search
        t = armijo_line_search(dz, mechanism, freebodies, freeids, jac, joint_idx, body_idx, 1e-4, 0.5, lineIter)
        
        if debug
            println("Armijo search: $t")
        end
        # Update the mechanism with the Newton step
        newton_step(dz, mechanism, freeids, t=t)

        z = get_maximal_configuration(mechanism, freeids)


        if !isnothing(vis)
            visualize(mechanism, vis=vis, visualize_floor=false)
        end

        norm0 = objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, zero_dz)
       
        if debug
            norm_2 = norm(get_residual(mechanism, freeids, z))
            println("Norm: $norm0, Norm_2: $norm_2")
        end

        if norm0 < ε
            residual_norm = norm(get_residual(mechanism, freeids, z))
            objective_function_val = objective_function(mechanism, freebodies,freeids, jac, joint_idx, body_idx, z, zero_dz)
            println("Converged: Iteration $i | Residual norm: $residual_norm | Objective function: $objective_function_val")
            break
        end
    end
end

# using Profile
# using ProfileView
function test_scissor_mechanism(vis)
    mechanism = initialize_mechanism()
    delete!(vis)
    visualize(mechanism, vis=vis, visualize_floor=false)
    freebodies, freeids = get_free_bodies(mechanism)

    z0 = randn(length(get_maximal_configuration(mechanism, freeids)))
    set_configuration!(mechanism, freeids, z0)

    initialize_joint_constraints(mechanism, z0, freeids=freeids, vis=vis)
end

@time test_scissor_mechanism(vis)

# Open a file to write the profiling results
open("profile_results.txt", "w") do io
    Profile.print(io)
end

function test_jansen(vis)
    # path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
    # mechanism = Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
    mechanism =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    for i in [6]
        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = Dojo.RotZ(pi)
    end
    delete!(vis)
    visualize(mechanism, vis=vis, visualize_floor=false)

    ids_leg1 = [body.id for body in mechanism.bodies if occursin("leg1", string(body.name))]
    ids_m = [body.id for body in mechanism.bodies if occursin("bar_m", string(body.name))]
    ids_a = [body.id for body in mechanism.bodies if occursin("bar_a", string(body.name))]
    ids_l = [body.id for body in mechanism.bodies if occursin("bar_l", string(body.name))]
    crank = [body.id for body in mechanism.bodies if occursin("crank", string(body.name))]
    crossbar = [body.id for body in mechanism.bodies if occursin("crossbar", string(body.name))]

    fixedids = [crank..., crossbar...]
    freebodies, freeids = get_free_bodies(mechanism, fixedids=fixedids)
    # println(length(freebodies))
    z0 = get_maximal_configuration(mechanism, freeids)
    # println(length(z0))
    # dz0 = zeros(length(freebodies)*6)
    # println(length(dz0))
    # res = get_residual(mechanism, freebodies, z0)
    # println(length(res))
    # jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freebodies, true)
    # println(size(jac))
    # update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx, true)
    # println(size(jac))
    # obj_func = (res+jac*dz0)'*(res+jac*dz0)
    # println(obj_func)
    # objective_function(mechanism, freebodies, z0, dz0)

    @time initialize_joint_constraints(mechanism, z0, fixedids=fixedids, newtonIter = 100, lineIter = 10, ε = 1e-5, debug=false)

    return mechanism
end


mechanism = test_jansen(vis)
visualize(mechanism, vis=vis, visualize_floor=false)


@save "mechanism_1leg_converge.jld2" mechanism

path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
mechanism_2leg =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
for i in [6]
    get_joint(mechanism_2leg, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism_2leg, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

    get_joint(mechanism_2leg, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism_2leg, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

    get_joint(mechanism_2leg, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = Dojo.RotZ(pi)
end


crank = [body.id for body in mechanism.bodies if occursin("crank", string(body.name))]
crossbar = [body.id for body in mechanism.bodies if occursin("crossbar", string(body.name))]

fixedids = [crank..., crossbar...]
initialize_constraints!(mechanism_2leg, fixedids=fixedids, newtonIter = 100, lineIter = 10, ε = 1e-5, debug=true, regularization=0.0)


set_configuration!(mechanism_1leg, mechanism_2leg.bodies, save_state)

mechanism_1leg = deepcopy(mechanism)
names_1leg = [body.name for body in mechanism_1leg.bodies]
names_2leg = [body.name for body in mechanism_2leg.bodies]
for name in names_1leg
    body_1leg = get_body(mechanism_1leg, name)
    body_2leg = get_body(mechanism_2leg, name)
    body_2leg.state.x2 = body_1leg.state.x2
    body_2leg.state.q2 = body_1leg.state.q2
end



leg1_ids = [body.id for body in mechanism.bodies if occursin("leg1", string(body.name))]
leg1_names = [body.name for body in mechanism.bodies if occursin("leg1", string(body.name))]
# convert leg1 to leg2 in all names 
leg2_names = [Symbol(replace(String(name), "leg1" => "leg2")) for name in leg1_names]
leg1_bodies = get_free_bodies(mechanism, freeids=leg1_ids)
leg1_z = get_maximal_configuration(mechanism, leg1_bodies)
# reset the y value for all bodies to 0.5
leg1_z[2:7:end] .= 0.5
set_configuration!(mechanism, leg1_bodies, leg1_z)
delete!(vis)
visualize(mechanism_2leg, vis=vis, visualize_floor=false, show_joint=true, show_frame=true)

#mirror the position and orientation of leg1 to leg2
leg2_ids = [get_body(mechanism, name).id for name in leg2_names]
leg2_bodies = get_free_bodies(mechanism, freeids=leg2_ids)
for (name1, name2) in zip(leg1_names, leg2_names)
    # if occursin("b_d_e", string(name1)) || occursin("g_h_i", string(name1))
    #     body1 = get_body(mechanism, name1)
    #     body2 = get_body(mechanism, name2)
    #     body2.state.x2 = Dojo.vector_rotate(body1.state.x2.* [-1.0, 1.0, 1.0], Dojo.RotY(pi)) #.* [-1.0, 1.0, 1.0]

    #     body2.state.q2 = body1.state.q2 * Dojo.RotY(pi)
    #     continue
    # end

    body1 = get_body(mechanism_2leg, name1)
    body2 = get_body(mechanism_2leg, name2)
    q_reflect = Quaternion(body1.state.q2.s, body1.state.q2.v1, body1.state.q2.v2, body1.state.q2.v3)
    body2.state.x2 = body1.state.x2 .* [-1.0, 1.0, 1.0]
    body2.state.q2 = Dojo.RotZ(pi)*q_reflect #body1.state.q2*Dojo.RotY(pi) #Quaternion(body1.state.q2.s, -body1.state.q2.v1, -body1.state.q2.v2, -body1.state.q2.v3)
end
delete!(vis)
visualize(mechanism_2leg, vis=vis, visualize_floor=false, show_joint=true, show_frame=true, joint_radius=0.05)

(Dojo.constraint(mechanism_2leg, get_joint(mechanism_2leg, :pair06_leg2_joint_k_c)))

get_joint(mechanism_2leg, :pair06_leg2_loop_f_g).rotational.orientation_offset = get_joint(mechanism_2leg, :pair06_leg1_loop_f_g).rotational.orientation_offset

get_joint(mechanism_2leg, :pair06_leg2_loop_b_c).rotational.orientation_offset = get_joint(mechanism_2leg, :pair06_leg1_loop_b_c).rotational.orientation_offset

get_joint(mechanism_2leg, :pair06_leg2_loop_a_c).rotational.orientation_offset = Dojo.RotZ(pi)

for joint_name in joint_names
    if norm((Dojo.constraint(mechanism_2leg, get_joint(mechanism_2leg, joint_name)))) > 1e-4

        println(joint_name)
        println(Dojo.constraint(mechanism_2leg, get_joint(mechanism_2leg, joint_name)))
        println(norm((Dojo.constraint(mechanism_2leg, get_joint(mechanism_2leg, joint_name)))))
    end
end


leg1_bde = get_body(mechanism, :pair06_leg1_bars_b_d_e)
leg2_bde = get_body(mechanism, :pair06_leg2_bars_b_d_e)

# leg2_z = get_maximal_configuration(mechanism, leg2_bodies)
# leg2_z[1:7:end] .= -leg1_z[1:7:end]
# leg2_z[2:7:end] .= leg1_z[2:7:end]
# leg2_z[3:7:end] .= leg1_z[3:7:end]
# leg2_z[4:7:end] .= leg1_z[4:7:end]
# leg2_z[5:7:end] .= -leg1_z[5:7:end]
# leg2_z[6:7:end] .= leg1_z[6:7:end]
# leg2_z[7:7:end] .= leg1_z[7:7:end]
# set_configuration!(mechanism, leg2_bodies, leg2_z)

bar_a_id = [body.id for body in mechanism.bodies if occursin("bar_a", string(body.name))]
bar_a_bodies = get_free_bodies(mechanism, freeids=bar_a_id)
bar_a_z = get_maximal_configuration(mechanism, bar_a_bodies)
bar_a_z[2:7:end] .= 0.5
bar_a_z[4:end] .= [sqrt(2)/2., 0.0, -sqrt(2)/2., 0.0]
bar_a_z[1] = 0.0
set_configuration!(mechanism, bar_a_bodies, bar_a_z)

bar_m_id = [body.id for body in mechanism.bodies if occursin("bar_m", string(body.name))]
bar_m_bodies = get_free_bodies(mechanism, freeids=bar_m_id)
bar_m_z = get_maximal_configuration(mechanism, bar_m_bodies)
bar_m_z[2:7:end] .= 0.5
bar_m_z[4:end] .= [1.0, 0., 0., 0.]
bar_m_z[1] = 0.0
set_configuration!(mechanism, bar_m_bodies, bar_m_z)

delete!(vis)
vis = Visualizer()
visualize(mechanism, vis=vis, visualize_floor=false)
freebodies = mechanism.bodies
freeids = Dojo.getid.(freebodies)
z0 = randn(length(get_maximal_configuration(mechanism, freebodies)))
# z0 = get_maximal_configuration(mechanism, mechanism.bodies)
set_configuration!(mechanism, freebodies, z0)
fixedids=[crank..., crossbar...]
freebodies = get_free_bodies(mechanism, fixedids=fixedids)
z0 = get_maximal_configuration(mechanism, freebodies)
initialize_joint_constraints(mechanism, z0, fixedids=[crank..., crossbar...], newtonIter = 10, lineIter = 10, ε = 1e-5, debug=true)








# mechanism = initialize_mechanism(30)

data = Dict(
    "j2_j1" => Dict("length" => 50.0, "position" => (-4.5067675485638965, 15.636048727421338), "angle" => -38.71463269805005, "label" => :pair06_leg1_bar_j),
    "j5_j1" => Dict("length" => 61.9, "position" => (-5.976053515786479, -22.75758508504058), "angle" => 47.33270859961152, "label" => :pair06_leg1_bar_k),
    "j5_j3" => Dict("length" => 39.3, "position" => (-32.476053515786475, -26.65758508504058), "angle" => 106.32687427036096, "label" => :pair06_leg1_bar_c),
    "j2_j3" => Dict("length" => 41.5, "position" => (-31.006767548563896, 11.736048727421338), "angle" => -109.69561783022645, "label" => :pair06_leg1_bars_b_d_e),
    # "j2_j4" => Dict("length" => 55.8, "position" => (-49.40395023903193, 19.70763383036941), "angle" => -155.51236186405146, "label" => "e"),
    # "j4_j3" => Dict("length" => 40.1, "position" => (-56.39718269046803, 0.17158510294807128), "angle" => -23.42730995038722, "label" => "d"),
    "j5_j6" => Dict("length" => 36.7, "position" => (-43.091810996493976, -36.784050200414455), "angle" => 151.5878748768938, "label" => :pair06_leg1_bars_g_h_i),
    "j4_j6" => Dict("length" => 39.4, "position" => (-67.01294017117553, -9.954880012425807), "angle" => -66.7342627136313, "label" => :pair06_leg1_bar_f),
    # "j5_j7" => Dict("length" => 49.0, "position" => (-35.056108777838844, -68.63605154810219), "angle" => -109.31585688104902, "label" => "i"),
    # "j7_j6" => Dict("length" => 65.7, "position" => (-51.195812742759855, -59.904931578435495), "angle" => 104.15926149980457, "label" => "h")
    "2j2_j1" => Dict("length" => 50.0, "position" => (4.5067675485638965, 15.636048727421338), "angle" => -180-38.71463269805005, "label" => :pair06_leg2_bar_j),
    "2j5_j1" => Dict("length" => 61.9, "position" => (5.976053515786479, -22.75758508504058), "angle" => 180-47.33270859961152, "label" => :pair06_leg2_bar_k),
    "2j5_j3" => Dict("length" => 39.3, "position" => (32.476053515786475, -26.65758508504058), "angle" => 180-106.32687427036096, "label" => :pair06_leg2_bar_c),
    "2j2_j3" => Dict("length" => 41.5, "position" => (31.006767548563896, 11.736048727421338), "angle" => -109.69561783022645, "label" => :pair06_leg2_bars_b_d_e),
    # "j2_j4" => Dict("length" => 55.8, "position" => (-49.40395023903193, 19.70763383036941), "angle" => -155.51236186405146, "label" => "e"),
    # "j4_j3" => Dict("length" => 40.1, "position" => (-56.39718269046803, 0.17158510294807128), "angle" => -23.42730995038722, "label" => "d"),
    "2j5_j6" => Dict("length" => 36.7, "position" => (43.091810996493976, -36.784050200414455), "angle" => -151.5878748768938, "label" => :pair06_leg2_bars_g_h_i),
    "2j4_j6" => Dict("length" => 39.4, "position" => (67.01294017117553, -9.954880012425807), "angle" => -180-66.7342627136313, "label" => :pair06_leg2_bar_f),
    # "j5_j7" => Dict("length" => 49.0, "position" => (-35.056108777838844, -68.63605154810219), "angle" => -109.31585688104902, "label" => "i"),
    # "j7_j6" => Dict("length" => 65.7, "position" => (-51.195812742759855, -59.904931578435495), "angle" => 104.15926149980457, "label" => "h")
)
delete!(vis)
for (key, value) in data
    print(value["label"])
    body = get_body(mechanism, Symbol(value["label"]))
    # print(value["position"][1]./100)
    print(value["position"][1] / 100.)
    body.state.x2 = [value["position"][1] / 100., body.state.x2[2],  value["position"][2] / 100.]
    # body.state.x2  =
    body.state.q2 = Quaternion(cos(deg2rad(value["angle"])/2.), 0.0, sin(deg2rad(value["angle"])/2.), 0.0)
end
visualize(mechanism, vis=vis, visualize_floor=false)

visualize(mechanism, vis=vis, visualize_floor=false)
initialize_joint_constraints(mechanism, fixedids=[crank..., crossbar...], newtonIter=100)
# 
initialize_constraints!(mechanism, fixedids=[crank..., crossbar...], newtonIter=100, debug=true, regularization=0.0)
visualize(mechanism, vis=vis, visualize_floor=false)
mechanism = initialize_mechanism()
path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
mechanism = Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
z = get_maximal_state(mechanism) # x, v, q, ω
set_maximal_state!(mechanism, z)
delete!(vis)

# vis = Visualizer()
# z = get_maximal_state(mechanism) # x, v, q, ω
# set_maximal_state!(mechanism, rand(length(z)))
ids_m = [body.id for body in mechanism.bodies if occursin("bar_m", string(body.name))]
ids_a = [body.id for body in mechanism.bodies if occursin("bar_a", string(body.name))]
ids_l = [body.id for body in mechanism.bodies if occursin("bar_l", string(body.name))]
crank = [body.id for body in mechanism.bodies if occursin("crank", string(body.name))]
crossbar = [body.id for body in mechanism.bodies if occursin("crossbar", string(body.name))]
visualize(mechanism, vis=vis, visualize_floor=false)
initialize_joint_constraints(mechanism, fixedids=[crank..., crossbar...], newtonIter=100)





res = Dojo.residual(mechanism)
X, Q = Dojo.displacement_jacobian_configuration(:child, mechanism.joints[1].translational, mechanism.bodies[1].state.x2, mechanism.bodies[1].state.q2, mechanism.bodies[2].state.x2, mechanism.bodies[2].state.q2, attjac=true)
Js, Ju = Dojo.get_maximal_gradients!(mechanism, get_maximal_state(mechanism), zeros(sum(Dojo.input_dimensions(mechanism))))
Dojo.get_minimal_gradients!(mechanism)



nu = input_dimension(mechanism)

for entry in mechanism.data_matrix.nzval # reset matrix
    entry.value .= 0.0
end
Dojo.jacobian_data!(mechanism.data_matrix, mechanism)
nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]
dimrow = length.(nodes)
dimcol = Dojo.data_dim.(nodes)
index_row = [1+sum(dimrow[1:i-1]):sum(dimrow[1:i]) for i in 1:length(dimrow)]
index_col = [1+sum(dimcol[1:i-1]):sum(dimcol[1:i]) for i in 1:length(dimcol)]

index_state = [index_col[body.id][[14:16; 8:10; 17:19; 11:13]] for body in mechanism.bodies] # ∂ x2 v15 q2 ω15
index_control = [index_col[joint.id][1:input_dimension(joint)] for joint in mechanism.joints] # ∂ u

datamat = full_matrix(mechanism.data_matrix, false, dimrow, dimcol)
solmat = full_matrix(mechanism.system)

# data Jacobian
data_jacobian = solmat \ datamat #TODO: use pre-factorization
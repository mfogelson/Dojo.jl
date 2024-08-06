using Dojo
using FiniteDiff
using LinearAlgebra
include("make_scissor.jl")
include("make_jansen.jl")

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
    con_jac = zeros(Float64, num_constraints, num_bodies)

    return con_jac, joint_idx, body_idx
end

function update_constraint_jacobian!(con_jac::AbstractMatrix, mechanism::Mechanism, freebodies::Vector{Body{Float64}}, joint_idx, body_idx, attjac=true)
    # Filling the constraint Jacobian
    for (i,(joint, j_idx)) in enumerate(zip(mechanism.joints, joint_idx))
        for (j,(body, b_idx)) in enumerate(zip(freebodies, body_idx))
            if body.id == joint.parent_id || body.id == joint.child_id
                con_jac[j_idx, b_idx] = Dojo.constraint_jacobian_configuration(mechanism, joint, body, attjac)
            end
        end
    end
end

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

# function update_stepvec!(stepvec::AbstractVector, res::AbstractVector, jac::AbstractMatrix; debug=false)
#         stepvec .= -jac \ res
# end

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

function get_residual(mechanism::Mechanism, freeids::Set{Int64}, z::AbstractVector{Float64})
    joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]
    joint_idx = Dojo.index_ranges(joint_dof)

    res = zeros(Float64, sum(joint_dof))
    set_configuration!(mechanism, freeids, z)

    for (idx, joint) in zip(joint_idx, mechanism.joints)
        res[idx] = Dojo.constraint(mechanism, joint)
    end
    
    return res
end


function objective_function(mechanism::Mechanism, freebodies::Vector{Body{Float64}}, freeids::Set{Int64}, jac::AbstractMatrix{Float64}, joint_idx, body_idx,  z::AbstractVector{Float64}, dz::AbstractVector{Float64})
    # println(size(z), size(dz))
    res = get_residual(mechanism, freeids, z)
    # jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freebodies, true)
    update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx, true)
    # println(size(res), size(jac))

    return 0.5*(res+jac*dz)'*(res+jac*dz)
end

function armijo_line_search(dz::AbstractVector{Float64}, mechanism::Mechanism, freebodies, freeids::Set{Int64}, jac, joint_idx, body_idx, α=1e-4, β=0.1, max_iter=100; debug=false)
    t = 1.0

    z = get_maximal_configuration(mechanism, freeids)
    # initial_obj = objective_function(mechanism, freebodies, z, dz.*t)
    res = get_residual(mechanism, freeids, z)
    initial_obj = norm(res)
    grad = (res + jac*dz)'*jac
    # grad =  FiniteDiff.finite_difference_gradient(_dz -> objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, _dz), dz)

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


function initialize_joint_constraints(mechanism::Mechanism{T}, z0::AbstractVector; fixedids = Int64[], freeids = Int64[], ε = 1e-5, newtonIter = 100, lineIter = 10, regularization = 1e-6, debug=false, vis=nothing, storage=Nothing) where T

    if isnothing(storage)
        storage = Storage(newtonIter, length(mechanism.bodies))
    end

    freebodies, freeids = get_free_bodies(mechanism, fixedids=fixedids, freeids=freeids)
    # set initial configuration 
    set_configuration!(mechanism, freeids, z0)

    # get the initial configuration
    z = get_maximal_configuration(mechanism, freeids)

    # Get the initial maximum violation of constraints
    # println(z0)
    # n = length(z)
    m = length(freebodies)*6
    dz = zeros(Float64, m)

    jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freeids, true)

    norm0 = objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, dz) #maximum(violations(mechanism))

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
        res = get_residual(mechanism, freeids, z)

        update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx, true)
        # Update the step vector
        update_stepvec!(dz, res, jac, debug=debug)
    

        # Perform Armijo line search
        t = armijo_line_search(dz, mechanism, freebodies, freeids, jac, joint_idx, body_idx, 1e-4, 0.5, lineIter, debug=debug)
        
        if debug
            println("Armijo search: $t")
        end
        # Update the mechanism with the Newton step
        newton_step(dz, mechanism, freeids, t=t)

        z = get_maximal_configuration(mechanism, freeids)

        Dojo.save_to_storage!(mechanism, storage, i)

        if !isnothing(vis)
            visualize(mechanism, vis=vis, visualize_floor=false)
        end

        norm0 = objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z, zero_dz)
       
        if debug
            norm_2 = norm(get_residual(mechanism, freeids, z))
            println("Norm: $norm0, Norm_2: $norm_2")
        end

        if norm0 < ε
            if i < newtonIter
                for j in i:newtonIter
                    Dojo.save_to_storage!(mechanism, storage, j)
                end
            end
            residual_norm = norm(get_residual(mechanism, freeids, z))
            objective_function_val = objective_function(mechanism, freebodies,freeids, jac, joint_idx, body_idx, z, zero_dz)
            println("Converged: Iteration $i | Residual norm: $residual_norm | Objective function: $objective_function_val")
            break
        end
    end
    residual_norm = norm(get_residual(mechanism, freeids, z))
    objective_function_val = objective_function(mechanism, freebodies,freeids, jac, joint_idx, body_idx, z, zero_dz)
    println("DID NOT CONVERGE: Residual norm: $residual_norm | Objective function: $objective_function_val")
    return storage
end

# ============================================================================ #
# Generate Jansen Data

mechanism = make_jansen_full()
fixed_ids = get_fixed_ids(mechanism)
freebodies, freeids = get_free_bodies(mechanism,fixedids=fixed_ids)
@time z0 = get_maximal_configuration(mechanism, freeids) #.+ 1e-5

newtonIter = 100
storage = Storage(newtonIter, length(mechanism.bodies))
initialize_joint_constraints(mechanism, z0, fixedids=fixed_ids, newtonIter = newtonIter, lineIter = 10, ε = 1e-8, debug=false, vis=nothing, storage=storage)

delete!(vis)
visualize(mechanism, storage, vis=vis, visualize_floor=false)
# ============================================================================ #


# ============================================================================ #
# Generate Scissor Data
for i in 2:10
    mechanism = initialize_mechanism(i)
    fixed_ids = [mechanism.bodies[1].id, mechanism.bodies[2].id]
    mechanism.bodies[1].state.x1 = [0.0, 0.0, 0.0]
    mechanism.bodies[1].state.q1 = Dojo.RotX(pi/2)*Dojo.RotY(pi/4)
    mechanism.bodies[1].state.x2 = [0.0, 0.0, 0.0]
    mechanism.bodies[1].state.q2 = Dojo.RotX(pi/2)*Dojo.RotY(pi/4)
    mechanism.bodies[2].state.x1 = [0.0, 0.0, 0.0]
    mechanism.bodies[2].state.q1 = Dojo.RotX(pi/2)*Dojo.RotY(-pi/4)
    mechanism.bodies[2].state.x2 = [0.0, 0.0, 0.0]
    mechanism.bodies[2].state.q2 = Dojo.RotX(pi/2)*Dojo.RotY(-pi/4)
    z0 = rand(length(z0))
    set_configuration!(mechanism, freeids, z0)
    storage = Storage(newtonIter, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=fixed_ids, newtonIter = newtonIter, lineIter = 10, ε = 1e-8, debug=false, vis=nothing, storage=storage)

    @save "scissor_cells_$(i).jld2" mechanism storage
end

delete!(vis)
visualize(mechanism, storage, vis=vis, visualize_floor=false, show_frame=true)
# ============================================================================ #


# ============================================================================ #
# Generate PET DATA
# for cells in 1:10
cells = 2
    mechanism = get_PET(αs[5], cells)
    delete!(vis)
    visualize(mechanism, vis=vis, visualize_floor=false, show_frame=true)
    long1 = get_body(mechanism, Symbol("long:c1:l1"))
    long2 = get_body(mechanism, Symbol("long:c1:l2"))
    fixed_ids = [long1.id, long2.id]
    vect = [0, 0, L2/2]
    q = Dojo.RotX(pi/2)*Dojo.RotY(αs[5]/2)
    c = cs[1]
    x = Dojo.vector_rotate(vect, q)
    long1.state.x1 = x
    long1.state.q1 = q
    long1.state.x2 = x
    long1.state.q2 = q

    vect = [0, 0, L2/2]
    q = Dojo.RotX(pi/2)*Dojo.RotY(-αs[5]/2)
    c = cs[1]
    x = Dojo.vector_rotate(vect, q)
    long2.state.x1 = x
    long2.state.q1 = q
    long2.state.x2 = x
    long2.state.q2 = q

    freebodies, freeids = get_free_bodies(mechanism, fixedids=fixed_ids)
    z0 = get_maximal_configuration(mechanism, freeids)
    z0 .+= 1e-5
    set_configuration!(mechanism, freeids, z0)
    storage = Storage(200, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=fixed_ids, newtonIter = 200, lineIter = 10, ε = 1e-6, debug=false, vis=nothing, storage=storage)
    delete!(vis)
    visualize(mechanism, storage, vis=vis, visualize_floor=false, show_frame=false)

    @save "PET_$(round(αs[5], digits=1))_$(cells)cells.jld2" mechanism storage
# end
# ============================================================================ #


# ============================================================================ #
# Old Code
function run()
    # mechanism = initialize_mechanism(4)
    mechanism = make_jansen_full()
    fixed_ids = get_fixed_ids(mechanism)
    # fixed_ids = [mechanism.bodies[1].id, mechanism.bodies[2].id]
    freebodies, freeids = get_free_bodies(mechanism,fixedids=fixed_ids)
    # freeids = Dojo.getid.(mechanism.bodies)
    @time z0 = get_maximal_configuration(mechanism, freeids) #.+ 1e-5
    # mechanism.bodies[1].state.x1 = [0.0, 0.0, 0.0]
    # mechanism.bodies[1].state.q1 = Dojo.RotX(pi/2)*Dojo.RotY(pi/4)
    # mechanism.bodies[1].state.x2 = [0.0, 0.0, 0.0]
    # mechanism.bodies[1].state.q2 = Dojo.RotX(pi/2)*Dojo.RotY(pi/4)
    # mechanism.bodies[2].state.x1 = [0.0, 0.0, 0.0]
    # mechanism.bodies[2].state.q1 = Dojo.RotX(pi/2)*Dojo.RotY(-pi/4)
    # mechanism.bodies[2].state.x2 = [0.0, 0.0, 0.0]
    # mechanism.bodies[2].state.q2 = Dojo.RotX(pi/2)*Dojo.RotY(-pi/4)
    # z0 = rand(length(z0))
    # set_configuration!(mechanism, freeids, z0)
    # z0[1:7] = [mechanism.bodies[1].state.x2..., Dojo.vector(mechanism.bodies[1].state.q2)...]
    # z0[8:14] = [mechanism.bodies[2].state.x2..., Dojo.vector(mechanism.bodies[2].state.q2)...]
    # @time out = get_maximal_state(mechanism)
    # @time jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freeids)
    # @time update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx)
    # @time set_configuration!(mechanism, freeids, z0)
    # @time res = get_residual(mechanism, freeids, z0)
    # dz = rand(length(freeids)*6)
    # @time grad = FiniteDiff.finite_difference_gradient(_dz -> objective_function(mechanism, freebodies, freeids, jac, joint_idx, body_idx, z0, _dz), dz)
    # grad_analytical = ((res+jac*dz)'*jac)'

    # path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
        # mechanism = Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    # path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
    # mechanism =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    # for i in [6]
    #     get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

    #     get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

    #     get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = Dojo.RotZ(pi)
    # end
    # delete!(vis)
    # visualize(mechanism, vis=vis, visualize_floor=false)

    # ids_leg1 = [body.id for body in mechanism.bodies if occursin("leg1", string(body.name))]
    # ids_m = [body.id for body in mechanism.bodies if occursin("bar_m", string(body.name))]
    # ids_a = [body.id for body in mechanism.bodies if occursin("bar_a", string(body.name))]
    # ids_l = [body.id for body in mechanism.bodies if occursin("bar_l", string(body.name))]
    # crank = [body.id for body in mechanism.bodies if occursin("crank", string(body.name))]
    # crossbar = [body.id for body in mechanism.bodies if occursin("crossbar", string(body.name))]

    # fixedids = [crank..., crossbar...]
    # freebodies, freeids = get_free_bodies(mechanism, fixedids=fixedids)
    # # println(length(freebodies))
    # z0 = get_maximal_configuration(mechanism, freeids)
    newtonIter = 200
    storage = Storage(newtonIter, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=fixed_ids, newtonIter = newtonIter, lineIter = 10, ε = 1e-5, debug=false, vis=nothing, storage=storage)

    # Profile.print(format=:flat, sortedby=:counts)
    delete!(vis)
    visualize(mechanism, storage, vis=vis, visualize_floor=false)


    vis = Visualizer()
    delete!(vis)
    visualize(mechanism, vis=vis, visualize_floor=false, show_frame=true)

    jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freeids)
    update_constraint_jacobian!(jac, mechanism, freebodies, joint_idx, body_idx)

    F = svd(jac, full=true, alg=LinearAlgebra.QRIteration())
    maximum(F.S)
    F.S
    save_state = get_maximal_configuration(mechanism, freeids)

    set_configuration!(mechanism, freeids, save_state)

    get_body(mechanism, :pair06_leg1_bar_f).id
    for i in 1:6
        println("############## LEG $i ##############")
        println("leg2_loop_f_g $(get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset)")
        println("leg1_loop_f_g $(get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset)")
        println("leg2_loop_b_c $(get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset)")
        println("leg1_loop_b_c $(get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset)")
        println("leg2_loop_a_c $(get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset)")
        println("leg1_loop_a_c $(get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_a_c")).rotational.orientation_offset)")
        println("leg1_joint_m_j $(get_joint(mechanism, Symbol("pair0$(i)_leg1_joint_m_j")).rotational.orientation_offset)")
        println("leg2_joint_m_j $(get_joint(mechanism, Symbol("pair0$(i)_leg2_joint_m_j")).rotational.orientation_offset)")

        # get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

        # get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = Dojo.RotZ(pi)
    end
    # using JLD2
    # @save "jansen_small.jld2" mechanism storage
    # @save "jansen_full.jld2" mechanism storage
end
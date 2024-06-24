using Dojo

function make_body(unit_thickness::Float64, unit_length::Float64, unit_mass::Float64, name::Symbol, color)
    return Cylinder(unit_thickness, unit_length, unit_mass, name=name, color=color)
end

function make_joint(parent::Body, child::Body, rotation_axis::Vector{Float64}, parent_vertex::Vector{Float64}, child_vertex::Vector{Float64}, name::Symbol)
    return JointConstraint(Revolute(parent, child, rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=name)
end

function initialize_mechanism()
    unit_thickness = 0.1
    unit_length = 1.0
    unit_mass = 1.0

    link1 = make_body(unit_thickness, unit_length, unit_mass, :link1, RGBA(1.0, 0, 0))
    link2 = make_body(unit_thickness, unit_length, unit_mass, :link2, RGBA(0, 1.0, 0))
    bodies = [link1, link2]
    joint1 = make_joint(link1, link2, [0, 1.0, 0.], zeros(3), zeros(3), :joint1)
    joints = [joint1]
    origin = Origin()
    mechanism = Mechanism(origin, bodies, joints)
end

function initialize_constraint_jacobian(mechanism, freebodies, attjac=true)
    # Getting the degrees of freedom for each joint
    joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]

    body_dof = attjac ? 6 : 7
    # Creating index ranges for the joints
    joint_idx = Dojo.index_ranges(joint_dof)

    # Calculating total number of constraints and bodies
    num_constraints = sum(joint_dof)
    num_bodies = body_dof*length(freebodies)

    # Creating index ranges for the bodies
    body_idx = Dojo.index_ranges([body_dof for _ in freebodies])

    # Creating the constraint matrix
    con_jac = zeros(num_constraints, num_bodies)

    return con_jac, joint_idx, body_idx
end

function update_constraint_jacobian!(con_jac, mechanism, freebodies, joint_idx, body_idx, attjac=true)
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

function update_stepvec!(stepvec, con_jac, mechanism)
    Dojo.rank(con_jac)
    F = svd(con_jac, full=true, alg=LinearAlgebra.QRIteration())
    rank = sum(F.S .> 1e-6)
    println("rank: $rank | full: $(size(F.V, 2))")
    V1 = @view F.V[:,1:rank]
    S1 = @view F.S[1:rank]
    U1 = @view F.U[:,1:rank]
    b =  Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
    stepvec[:] .= -V1*Diagonal(1.0 ./ S1)*U1'*b
end

# 0.5*(b+con_jac*stepvec)'*(b+con_jac*stepvec)
# b =  Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))

# Function to compute the objective function (user-defined)
function objective_function(mechanism)
    res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))

    return 0.5*res'*res
end

# Function to perform Armijo line search
function armijo_line_search(stepvec, mechanism, freebodies, α=1e-4, β=0.1, max_iter=100)
    t = 1.0
    initial_obj = objective_function(mechanism)
    
    for i in 1:max_iter
        # Create a temporary mechanism to test the step
        temp_mechanism = deepcopy(mechanism)
        newton_step(stepvec .* t, temp_mechanism, freebodies)
        
        new_obj = objective_function(temp_mechanism)
        println("New obj: $new_obj | Initial obj: $(initial_obj) | Update: $(α * t * dot(stepvec, stepvec)) | α: $α | β: $β | t: $t")
        res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
        if new_obj <= initial_obj + α * t * dot(stepvec, stepvec)
            println("Line search converged")
            return t
        else
            t *= β
        end
    end
    println("Line search did not converge")
    return t  # Return the step size found
end

# Function to update the mechanism with a bounded angular step and Armijo line search
function newton_step(stepvec, mechanism, freebodies)

    # TODO Bound the angular step vector
    # max_angle = π / 4  # Example bound, adjust as necessary
    shift = 0
    freeids = Dojo.getid.(freebodies)
    for body in mechanism.bodies
        if !in(body.id, freeids)
            continue
        end
        body.state.x2 = body.state.x1 + stepvec[(1:3) .+ shift]

        # Normalize the step vector quaternion
        if norm(stepvec[(4:6) .+ shift]) > 1.0
            step_q = stepvec[(4:6) .+ shift] ./ norm(stepvec[(4:6) .+ shift])
        else
            step_q = stepvec[(4:6) .+ shift]
        end

        w = sqrt(1 - min(1,  norm(step_q)^2))

        body.state.q2 = body.state.q1 * Quaternion(w, (step_q)...)

        body.state.q2 = body.state.q2 / norm(body.state.q2)

        shift += 6
    end
end

function initialize_joint_constraints(mechanism::Mechanism{T}; fixedids = Int64[], freeids = Int64[], ε = 1e-5, newtonIter = 100, lineIter = 10, regularization = 1e-6, debug=false) where T
    # Initialize the array of free bodies
    freebodies = Body[]

    # Check if both freeids and fixedids are specified
    if !isempty(fixedids) && !isempty(freeids)
        error("Specify either free or fixed bodies, not both.")
    elseif !isempty(fixedids)  # Only fixedids are specified
        freeids = setdiff(Dojo.getid.(mechanism.bodies),fixedids)
        freebodies = [get_body(mechanism, id) for id in freeids]
    elseif !isempty(freeids)  # Only freeids are specified
        freebodies = [get_body(mechanism, id) for id in freeids]
    else  # Neither are specified, consider all bodies free
        freeids = Dojo.getid.(mechanism.bodies)
        freebodies = [get_body(mechanism, id) for id in freeids]
    end

    # Get the initial maximum violation of constraints
    norm0 = objective_function(mechanism) #maximum(violations(mechanism))
    norm1 = norm0

    con_jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, freebodies, true)
    stepvec = zeros(size(con_jac, 2))

    # Newton-Raphson iterations
    for i in Base.OneTo(newtonIter)
        println("Iter: $i | Norm: $norm0")

        # Copy state variables from previous step
        for body in freebodies
            body.state.x1 = 1.0*body.state.x2
            body.state.q1 = 1.0*body.state.q2
        end

        # Update the constraint Jacobian
        update_constraint_jacobian!(con_jac, mechanism, freebodies, joint_idx, body_idx, true)

        # Update the step vector
        update_stepvec!(stepvec, con_jac, mechanism)

        # Perform Armijo line search
        t = armijo_line_search(stepvec, mechanism, freebodies, 1e-4, 0.1, lineIter)
        println("Armijo search: $t")
        # Update the mechanism with the Newton step
        newton_step(stepvec * t, mechanism, freebodies)
        visualize(mechanism, vis=vis, visualize_floor=false)


        norm0 = objective_function(mechanism)
        if norm0 < ε
            println("Converged")
            break
        end
    end
end

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
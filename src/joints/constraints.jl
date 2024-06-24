"""
    JointConstraint{T} <: Constraint{T}

    constraint restricting translational and rotational degrees of freedom between two Body objects.

    id: a unique identifying number
    name: a unique identifying name
    translational: Translational
    rotational: Rotational
    spring: flag for joint springs on
    damper: flag for joint dampers on
    parent_id: identifying number for parent Body{T}
    child_id: identifying number for child Body{T}
    minimal_index: indices for minimal coordinates
    impulses: joint impulses that maintain constraint between two Body{T} objects
"""
mutable struct JointConstraint{T,N,Nc,TJ,RJ} <: Constraint{T,N}
    # ID
    id::Int64
    name::Symbol

    # joint constraints
    translational::TJ
    rotational::RJ

    # springs and dampers
    spring::Bool
    damper::Bool

    # neighbor IDs
    parent_id::Int
    child_id::Int

    # indices
    minimal_index::SVector{Nc,SVector{2,Int64}} # indices for minimal coordinates, assumes joints # Nc = 2 THIS IS SPECIAL CASED

    # impulses
    impulses::Vector{SVector{N,T}}

    function JointConstraint(data;
        name::Symbol=Symbol("joint_" * randstring(4)))

        @assert data[1][2] == data[2][2] # check parent ids
        @assert data[1][3] == data[2][3] # check child ids

        # joints
        translational = data[1][1]
        rotational = data[2][1]

        # IDs
        parent_id = data[1][2]
        child_id = data[1][3]

        # data dype
        T = typeof(data[1][1]).parameters[1]

        # set springs & dampers off
        spring = false
        damper = false

        minimal_index = Vector{Int64}[]
        N = 0
        for joint_data in data
            joint = joint_data[1]

            # set spring & damper on
            joint.spring != 0 && (spring = true)
            joint.damper != 0 && (damper = true)

            # minimal-coordaintes indices
            Nλ = joint_length(joint)
            Nset = impulses_length(joint)
            if isempty(minimal_index)
                push!(minimal_index, [1;3-Nλ])
            else
                push!(minimal_index, [last(minimal_index)[2]+1; last(minimal_index)[2]+3-Nλ])
            end
            N += Nset
        end

        Nc = 2
        impulses = [zeros(T, N) for i=1:2]

        return new{T,N,Nc,typeof(translational),typeof(rotational)}(getGlobalID(), name, translational, rotational, spring, damper, parent_id, child_id, minimal_index, impulses)
    end
end

function Base.show(io::IO, mime::MIME{Symbol("text/plain")}, constraint::JointConstraint)
    summary(io, constraint)
    println(io, "")
    println(io, "id:            "*string(constraint.id))
    println(io, "name:          "*string(constraint.name))
    println(io, "spring:        "*string(constraint.spring))
    println(io, "damper:        "*string(constraint.damper))
    println(io, "parent_id:     "*string(constraint.parent_id))
    println(io, "child_id:      "*string(constraint.child_id))
    println(io, "minimal_index: "*string(constraint.minimal_index))
    println(io, "impulses:      "*string(constraint.impulses))
end

# constraints
# @generated function constraint(mechanism, joint::JointConstraint)
#     pbody = :(get_body(mechanism, joint.parent_id))
#     cbody = :(get_body(mechanism, joint.child_id))
#     tra = :(constraint(joint.translational,
#         $pbody, $cbody,
#         joint.impulses[2][joint_impulse_index(joint,1)], mechanism.μ, mechanism.timestep))
#     rot = :(constraint(joint.rotational,
#         $pbody, $cbody,
#         joint.impulses[2][joint_impulse_index(joint,2)], mechanism.μ, mechanism.timestep))
#     return :(svcat($tra, $rot))
# end

function constraint(mechanism, joint::JointConstraint)
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    tra = constraint(joint.translational, pbody, cbody, joint.impulses[2][joint_impulse_index(joint,1)], mechanism.μ, mechanism.timestep)
    rot = constraint(joint.rotational, pbody, cbody, joint.impulses[2][joint_impulse_index(joint,2)], mechanism.μ, mechanism.timestep)
    return svcat(tra, rot)
end

# # constraints Jacobians
# @generated function constraint_jacobian(joint::JointConstraint)
#     tra = :(constraint_jacobian(joint.translational, joint.impulses[2][joint_impulse_index(joint, 1)]))
#     rot = :(constraint_jacobian(joint.rotational, joint.impulses[2][joint_impulse_index(joint, 2)]))
#     return :(cat($tra, $rot, dims=(1,2)))
# end
function constraint_jacobian(joint::JointConstraint; reg::Float64=Dojo.REG)
    tra = constraint_jacobian(joint.translational, joint.impulses[2][joint_impulse_index(joint, 1)], reg=reg)
    rot = constraint_jacobian(joint.rotational, joint.impulses[2][joint_impulse_index(joint, 2)], reg=reg)
    return diagonal_cat(tra, rot)
end

@generated function constraint_jacobian_configuration(mechanism, joint::JointConstraint, body::Body)
    relative = :(body.id == joint.parent_id ? :parent : :child)
    pbody = :(get_body(mechanism, joint.parent_id))
    cbody = :(get_body(mechanism, joint.child_id))
    tra = :(constraint_jacobian_configuration($relative,
        joint.translational,
        $pbody, $cbody,
        joint.impulses[2][joint_impulse_index(joint, 1)], mechanism.timestep))
    rot = :(constraint_jacobian_configuration($relative,
        joint.rotational,
        $pbody, $cbody,
        joint.impulses[2][joint_impulse_index(joint, 2)], mechanism.timestep))
    return :(vcat($tra, $rot))
end


@generated function constraint_jacobian_configuration(mechanism, joint::JointConstraint, body::Body, attjac::Bool)
    relative = :(body.id == joint.parent_id ? :parent : :child)
    pbody = :(get_body(mechanism, joint.parent_id))
    cbody = :(get_body(mechanism, joint.child_id))
    tra = :(constraint_jacobian_configuration($relative,
        joint.translational,
        $pbody, $cbody,
        joint.impulses[2][joint_impulse_index(joint, 1)], mechanism.timestep, attjac))
    rot = :(constraint_jacobian_configuration($relative,
        joint.rotational,
        $pbody, $cbody,
        joint.impulses[2][joint_impulse_index(joint, 2)], mechanism.timestep, attjac))
    return :(vcat($tra, $rot))
end

# impulses
function impulses!(mechanism, body::Body, joint::JointConstraint{T,Nλ}) where {T,Nλ}
    (Nλ > 0) && (body.state.d -= impulse_map(mechanism, joint, body) * joint.impulses[2])
    joint.spring && (body.state.d -= spring_impulses(mechanism, joint, body))
    joint.damper && (body.state.d -= damper_impulses(mechanism, joint, body))
    return
end

function impulse_map(mechanism, joint::JointConstraint, body::Body)
    relative = body.id == joint.parent_id ? :parent : :child
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    tra = impulse_map(relative, joint.translational,
        pbody, cbody,
        joint.impulses[2][joint_impulse_index(joint, 1)])
    rot = impulse_map(relative, joint.rotational,
        pbody, cbody,
        joint.impulses[2][joint_impulse_index(joint, 2)])
    return hcat(tra, rot)
end

# @generated function impulse_map(mechanism, joint::JointConstraint, body::Body)
#     relative = :(body.id == joint.parent_id ? :parent : :child)
#     pbody = :(get_body(mechanism, joint.parent_id))
#     cbody = :(get_body(mechanism, joint.child_id))
#     tra = :(impulse_map($relative, joint.translational,
#         $pbody, $cbody,
#         joint.impulses[2][joint_impulse_index(joint, 1)]))
#     rot = :(impulse_map($relative, joint.rotational,
#         $pbody, $cbody,
#         joint.impulses[2][joint_impulse_index(joint, 2)]))
#     return :(hcat($tra, $rot))
# end

# impulses Jacobians
function impulses_jacobian_velocity!(mechanism, body::Body, joint::JointConstraint)

    # relative
    relative = (body.id == joint.parent_id ? :parent : (body.id == joint.child_id ? :child : error()))

    # time step
    timestep= mechanism.timestep

    # bodies
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)

    # springs
    joint.spring && (body.state.D -= spring_jacobian_velocity(relative, relative, joint.translational, pbody, cbody, timestep))
    joint.spring && (body.state.D -= spring_jacobian_velocity(relative, relative, joint.rotational, pbody, cbody, timestep))

    # dampers
    joint.damper && (body.state.D -= damper_jacobian_velocity(relative, relative, joint.translational, pbody, cbody, timestep))
    joint.damper && (body.state.D -= damper_jacobian_velocity(relative, relative, joint.rotational, pbody, cbody, timestep))

    return
end

# off-diagonal Jacobians
function off_diagonal_jacobians(mechanism, body::Body, joint::JointConstraint)
    return -impulse_map(mechanism, joint, body), constraint_jacobian_configuration(mechanism, joint, body) * integrator_jacobian_velocity(body, mechanism.timestep)
end

function off_diagonal_jacobians(mechanism, joint::JointConstraint, body::Body)
    return constraint_jacobian_configuration(mechanism, joint, body) * integrator_jacobian_velocity(body, mechanism.timestep), -impulse_map(mechanism, joint, body)
end

function off_diagonal_jacobians(mechanism, pbody::Body, cbody::Body)
    # time step
    timestep = mechanism.timestep

    # dimensions
    Ne = length(mechanism.joints)
    Nb = length(mechanism.bodies)
    Nc = length(mechanism.contacts)

    # Jacobian
    jacobian_parent_child = szeros(6, 6)
    jacobian_child_parent = szeros(6, 6)

    for connectionid in connections(mechanism.system, pbody.id)
        # joints
        if connectionid <= Ne
            joint = get_node(mechanism, connectionid)
            if pbody.id == joint.parent_id
                for element in (joint.translational, joint.rotational)
                    if cbody.id == joint.child_id
                        joint.damper && (jacobian_parent_child -= damper_jacobian_velocity(:parent, :child, element, pbody, cbody, timestep))
                        joint.damper && (jacobian_child_parent -= damper_jacobian_velocity(:child, :parent, element, pbody, cbody, timestep))
                    end
                end
            elseif cbody.id == joint.parent_id
                for element in (joint.translational, joint.rotational)
                    if pbody.id == joint.child_id
                        joint.damper && (jacobian_parent_child -= damper_jacobian_velocity(:parent, :child, element, cbody, pbody, timestep))
                        joint.damper && (jacobian_child_parent -= damper_jacobian_velocity(:child, :parent, element, cbody, pbody, timestep))
                    end
                end
            end
        end

        # contacts
        if connectionid > Ne + Nb
            contact = get_node(mechanism, connectionid)
            if pbody.id == contact.parent_id
                if cbody.id == contact.child_id
                    Jpc = impulse_map_jacobian(:parent, :child, contact.model,
                            pbody,
                            cbody,
                            contact.impulses[2],
                            mechanism.timestep) * integrator_jacobian_velocity(cbody, timestep)

                    Jcp = impulse_map_jacobian(:child, :parent, contact.model,
                            pbody,
                            cbody,
                            contact.impulses[2],
                            mechanism.timestep) * integrator_jacobian_velocity(pbody, timestep)
                    # impulse_map_jacobian_configuration(mechanism, body, contact) * integrator_jacobian_velocity(body, timestep)
                    # impulse_map(mechanism, contact, body) * contact.impulses[2]
                    jacobian_parent_child -= Jpc#damper_jacobian_velocity(:parent, :child, element, pbody, cbody, timestep)
                    jacobian_child_parent -= Jcp#damper_jacobian_velocity(:child, :parent, element, pbody, cbody, timestep)
                end
            elseif cbody.id == contact.parent_id
                if pbody.id == contact.child_id
                    Jpc = impulse_map_jacobian(:parent, :child, contact.model,
                            cbody,
                            pbody,
                            contact.impulses[2],
                            mechanism.timestep) * integrator_jacobian_velocity(pbody, timestep)

                    Jcp = impulse_map_jacobian(:child, :parent, contact.model,
                            cbody,
                            bbody,
                            contact.impulses[2],
                            mechanism.timestep) * integrator_jacobian_velocity(cbody, timestep)

                    jacobian_parent_child -= Jpc #damper_jacobian_velocity(:parent, :child, element, cbody, pbody, timestep)
                    jacobian_child_parent -= Jcp #damper_jacobian_velocity(:child, :parent, element, cbody, pbody, timestep)
                end
            end
        end
    end

    return jacobian_parent_child, jacobian_child_parent
end

# linear system
function set_matrix_vector_entries!(mechanism, matrix_entry::Entry, vector_entry::Entry, joint::JointConstraint; reg::Float64=Dojo.REG)
    matrix_entry.value = constraint_jacobian(joint, reg=reg)
    vector_entry.value = -constraint(mechanism, joint)
end

# springs
function spring_impulses(mechanism, joint::JointConstraint{T}, body::Body{T};
    unitary::Bool=false) where T

    relative = (body.id == joint.parent_id ? :parent : :child)
    impulses = szeros(T,6)

    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)

    impulses += spring_impulses(relative, joint.translational,
        pbody,
        cbody,
        mechanism.timestep,
        unitary=unitary)

    impulses += spring_impulses(relative, joint.rotational,
        pbody,
        cbody,
        mechanism.timestep,
        unitary=unitary)

    return impulses
end

# dampers
function damper_impulses(mechanism, joint::JointConstraint{T}, body::Body;
    unitary::Bool=false) where T

    relative = (body.id == joint.parent_id ? :parent : :child)
    impulses = szeros(T,6)

    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)

    impulses += damper_impulses(relative, joint.translational,
        pbody,
        cbody,
        mechanism.timestep,
        unitary=unitary)

    impulses += damper_impulses(relative, joint.rotational,
        pbody,
        cbody,
        mechanism.timestep,
        unitary=unitary)

    return impulses
end

# inputs
function set_input!(joint::JointConstraint{T,N,Nc}, input::AbstractVector) where {T,N,Nc}
    @assert length(input) == input_dimension(joint)
    # translational
    r_idx = SUnitRange(joint.minimal_index[1][1], joint.minimal_index[1][2])
    length(r_idx) > 0 && set_input!(joint.translational, input[r_idx])
    # rotational
    r_idx = SUnitRange(joint.minimal_index[2][1], joint.minimal_index[2][2])
    length(r_idx) > 0 && set_input!(joint.rotational, input[r_idx])
    return
end

function add_input!(joint::JointConstraint{T,N,Nc}, input::AbstractVector) where {T,N,Nc}
    @assert length(input) == input_dimension(joint)
    add_input!(joint.translational, input[SUnitRange(joint.minimal_index[1][1], joint.minimal_index[1][2])])
    add_input!(joint.rotational, input[SUnitRange(joint.minimal_index[2][1], joint.minimal_index[2][2])])
    return
end

@generated function input_jacobian_control(mechanism, joint::JointConstraint{T,N,Nc}, body::Body) where {T,N,Nc}
    relative = :(body.id == joint.parent_id ? :parent : :child)
    pbody = :(get_body(mechanism, joint.parent_id))
    cbody = :(get_body(mechanism, joint.child_id))
    rot = :(input_jacobian_control($relative, joint.translational, $pbody, $cbody, mechanism.input_scaling))
    tra = :(input_jacobian_control($relative, joint.rotational, $pbody, $cbody, mechanism.input_scaling))
    return :(hcat($rot, $tra))
end

function input_impulse!(joint::JointConstraint{T,N,Nc}, mechanism, clear::Bool=true) where {T,N,Nc}
    pbody = get_body(mechanism, joint.parent_id)
    cbody = get_body(mechanism, joint.child_id)
    input_impulse!(joint.translational, pbody, cbody, mechanism.input_scaling, clear)
    input_impulse!(joint.rotational, pbody, cbody, mechanism.input_scaling, clear)
    return
end

# minimal
@generated function minimal_coordinates(mechanism, joint::JointConstraint{T,N,Nc}) where {T,N,Nc}
    pbody = :(get_body(mechanism, joint.parent_id))
    cbody = :(get_body(mechanism, joint.child_id))
    tra = :(minimal_coordinates(joint.translational, $pbody, $cbody))
    rot = :(minimal_coordinates(joint.rotational, $pbody, $cbody))
    return :(svcat($tra, $rot))
end

@generated function minimal_velocities(mechanism, joint::JointConstraint{T,N,Nc}) where {T,N,Nc}
    pbody = :(get_body(mechanism, joint.parent_id))
    cbody = :(get_body(mechanism, joint.child_id))
    tra = :(minimal_velocities(joint.translational, $pbody, $cbody, mechanism.timestep))
    rot = :(minimal_velocities(joint.rotational, $pbody, $cbody, mechanism.timestep))
    return :(svcat($tra, $rot))
end

################################################################################
# Utilities
################################################################################
function get_joint_impulses(joint::JointConstraint{T,N,Nc}, i::Int) where {T,N,Nc}
    n1 = 1
    for j = 1:i-1
        n1 += impulses_length((joint.translational, joint.rotational)[j])
    end
    n2 = n1 - 1 + impulses_length((joint.translational, joint.rotational)[i])

    λi = SVector{n2-n1+1,T}(joint.impulses[2][SUnitRange(n1,n2)])
    return λi
end

function joint_impulse_index(joint::JointConstraint{T,N,Nc}, i::Int) where {T,N,Nc}
    s = 0
    for j = 1:i-1
        element = (joint.translational, joint.rotational)[j]
        s += impulses_length(element)
    end
    joint_impulse_index((joint.translational, joint.rotational)[i], s)
end

# function reset!(joint::JointConstraint{T,N,Nc};
#     scale::T=1.0) where {T,N,Nc}
#     λ = []
#     for (i, element) in enumerate((joint.translational, joint.rotational))
#         Nλ = joint_length(element)
#         Nb = limits_length(element)
#         push!(λ, [scale * sones(2Nb); szeros(Nλ)])
#     end
#     joint.impulses[1] = vcat(λ...)
#     joint.impulses[2] = vcat(λ...)
#     return
# end

function reset!(joint::JointConstraint{T,N,Nc}; scale::T=1.0) where {T,N,Nc}
    Nλ_tra = joint_length(joint.translational)
    Nb_tra = limits_length(joint.translational)
    Nλ_rot = joint_length(joint.rotational)
    Nb_rot = limits_length(joint.rotational)
    joint.impulses[1] = [scale * sones(2Nb_tra); szeros(Nλ_tra); scale * sones(2Nb_rot); szeros(Nλ_rot)]
    joint.impulses[2] = [scale * sones(2Nb_tra); szeros(Nλ_tra); scale * sones(2Nb_rot); szeros(Nλ_rot)]
    return
end

function input_dimension(joint::JointConstraint{T,N,Nc};
    ignore_floating_base::Bool=false) where {T,N,Nc}
    ignore_floating_base && (N == 0) && return 0
    N̄ = 0
    N̄ = input_dimension(joint.translational) + input_dimension(joint.rotational)
    return N̄
end

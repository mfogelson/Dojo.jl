################################################################################
# Spring Impulses
################################################################################

function spring_force(relative::Symbol, joint::Translational{T},
    xa::AbstractVector, qa::Quaternion,
    xb::AbstractVector, qb::Quaternion;
    unitary::Bool=false) where T

    spring = unitary ? 1.0 : joint.spring
    distance = joint.spring_offset .- minimal_coordinates(joint, xa, qa, xb, qb)
    force = spring * zerodimstaticadjoint(nullspace_mask(joint)) * distance

    return [force; szeros(T, 3)]
end

function spring_impulses(relative::Symbol, joint::Translational, pbody::Node, cbody::Node, timestep; unitary::Bool=false)
    spring_impulses(relative, joint, 
        current_configuration(pbody.state)..., 
        current_configuration(cbody.state)..., 
        timestep, unitary=unitary)
end

function spring_impulses(relative::Symbol, joint::Translational,
    xa::AbstractVector, qa::Quaternion,
    xb::AbstractVector, qb::Quaternion,
    timestep; unitary::Bool=false)
    timestep * impulse_transform(relative, joint, xa, qa, xb, qb) * spring_force(relative, joint, xa, qa, xb, qb; unitary=unitary)[SA[1;2;3]]
end

spring_impulses(relative::Symbol, joint::Translational{T,3}, pbody::Node, cbody::Node, timestep; unitary::Bool=false) where T = szeros(T, 6)

################################################################################
# Spring Jacobians
################################################################################

function spring_force_jacobian_configuration(jacobian_relative::Symbol,
        joint::Translational{T},
        xa::AbstractVector, qa::Quaternion,
        xb::AbstractVector, qb::Quaternion;
        unitary::Bool=false, 
        attjac=true) where T
    spring = unitary ? 1.0 : joint.spring
    return -spring * zerodimstaticadjoint(nullspace_mask(joint)) * minimal_coordinates_jacobian_configuration(jacobian_relative, joint, xa, qa, xb, qb, attjac=attjac)
end

function spring_jacobian_configuration(relative::Symbol, jacobian_relative::Symbol,
    joint::Translational{T},
    xa::AbstractVector, qa::Quaternion,
    xb::AbstractVector, qb::Quaternion,
    timestep::T; 
    unitary::Bool=false, 
    attjac=true) where T

    force = spring_force(relative, joint, xa, qa, xb, qb, unitary=unitary)[SA[1;2;3]]
    
    J = impulse_transform(relative, joint, xa, qa, xb, qb) *
        spring_force_jacobian_configuration(jacobian_relative, joint, xa, qa, xb, qb, unitary=unitary, attjac=attjac)
    J += impulse_transform_jacobian(relative, jacobian_relative, joint, xa, qa, xb, qb, force, attjac=attjac)

    return timestep * J
end

function spring_jacobian_configuration(relative::Symbol, jacobian::Symbol,
    joint::Translational,
    pbody::Node, cbody::Node,
    timestep::T; 
    attjac::Bool=true, 
    unitary=false) where T
    return spring_jacobian_configuration(relative, jacobian, 
        joint, 
        current_configuration(pbody.state)..., 
        current_configuration(cbody.state)..., 
        timestep; unitary=false, attjac=attjac)
end

spring_jacobian_configuration(relative::Symbol, jacobian::Symbol, joint::Translational{T,3}, pbody::Node, cbody::Node, timestep::T; attjac::Bool = true, unitary::Bool=false) where T = attjac ? szeros(T, 6, 6) : szeros(T, 6, 7)
spring_jacobian_velocity(relative::Symbol, jacobian::Symbol, joint::Translational, pbody::Node, cbody::Node, timestep::T, unitary::Bool=false) where T = szeros(T, 6, 6)


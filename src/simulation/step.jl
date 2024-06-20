"""
    step!(mechanism::Mechanism{T}, z::Vector{T}, u::Vector{T}; opts)

    simulate mechanism for one time step provided maximal coordinates

    mechanism: Mechanism
    z: maximal state 
    u: inputs
    opts: SolverOptions
"""
function step!(mechanism::Mechanism{T}, z::Vector{T}, u::Vector{T}; 
    opts=SolverOptions{T}()) where T
    
    # set state
    set_maximal_state!(mechanism, z)

    # set control
    set_input!(mechanism, u)

    # solve the 1-step simulation problem
    mehrotra!(mechanism, opts=opts)
    for body in mechanism.bodies 
        update_state!(body, mechanism.timestep) 
    end

    # extract the next state
    z_next = get_next_state(mechanism)
    
    return z_next
end

function step!(mechanism::Mechanism{T}, z::Vector{T}, u::Vector{T}, m::Vector{T}, d::Vector{T}; 
    opts=SolverOptions{T}()) where T
    # set_mass and set_damper 
    for (i, body) in enumerate(mechanism.bodies)
        set_mass!(body, m[i])
    end

    for (i, joint) in (mechanism.joints)
        set_damper!([joint], d[i])
    end

    # set state
    set_maximal_state!(mechanism, z)

    # set control
    # set_input!(mechanism, u)
    top = get_body(mechanism, Symbol("plate_3"))
    set_external_force!(top, force=u)

    # solve the 1-step simulation problem
    mehrotra!(mechanism, opts=opts)
    for body in mechanism.bodies 
        update_state!(body, mechanism.timestep) 
    end

    # extract the next state
    z_next = get_next_state(mechanism)
    
    return z_next
end

"""
    step_minimal_coordinates!(mechanism::Mechanism{T}, x::Vector{T}, u::Vector{T}; opts)

    simulate mechanism for one time step provided minimal coordinates

    mechanism: Mechanism
    x: minimal state 
    u: inputs
    opts: SolverOptions
"""
function step_minimal_coordinates!(mechanism::Mechanism{T}, x::Vector{T}, u::Vector{T}; 
    opts=SolverOptions{T}()) where T
    
    # set state
    set_maximal_state!(mechanism, minimal_to_maximal(mechanism, x))

    # set control
    set_input!(mechanism, u)

    # solve the 1-step simulation problem
    mehrotra!(mechanism, opts=opts)
    for body in mechanism.bodies 
        update_state!(body, mechanism.timestep) 
    end

    # extract the next state
    x_next = maximal_to_minimal(mechanism, get_next_state(mechanism))
    
    return x_next
end
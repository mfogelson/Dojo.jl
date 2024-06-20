function set_velocity_solution!(body::Body)
    state = body.state
    state.vsol[1] = state.v15
    state.vsol[2] = state.v15
    state.ωsol[1] = state.ω15
    state.ωsol[2] = state.ω15
end

function set_previous_configuration!(body::Body{T}, timestep::T) where {T}
    x2, v15, q2, ω15 = initial_configuration_velocity(body.state)
    body.state.x1 = next_position(x2, -v15, timestep)
    body.state.q1 = next_orientation(q2, -ω15, timestep)
end

function initialize_state!(body::Body{T}, timestep) where T
    set_previous_configuration!(body, timestep)
    state = body.state
    state.JF2 = szeros(T,3)
    state.Jτ2 = szeros(T,3)
end

function update_state!(body::Body{T}, timestep) where T
    state = body.state

    state.x1 = state.x2
    state.q1 = state.q2

    state.v15 = state.vsol[2]
    state.ω15 = state.ωsol[2]

    state.x2 = next_position(state.x2, state.vsol[2], timestep)
    state.q2 = next_orientation(state.q2, state.ωsol[2], timestep)

    state.JF2 = szeros(T,3)
    state.Jτ2 = szeros(T,3)
end

# maximal
function set_maximal_configurations!(body::Body; 
    x::AbstractVector=SA[0.0; 0.0; 0.0], 
    q::Quaternion=one(Quaternion))

    body.state.x2 = x
    body.state.q2 = q

    return body.state.x2, body.state.q2
end

function set_maximal_velocities!(body::Body; 
    v::AbstractVector=SA[0.0; 0.0; 0.0], 
    ω::AbstractVector=SA[0.0; 0.0; 0.0])

    body.state.v15 = v
    body.state.ω15 = ω

    return body.state.v15, body.state.ω15
end

function set_maximal_configurations!(pbody::Node, cbody::Body;
    parent_vertex::AbstractVector=SA[0.0; 0.0; 0.0], 
    child_vertex::AbstractVector=SA[0.0; 0.0; 0.0],
    Δx::AbstractVector=SA[0.0; 0.0; 0.0], 
    Δq::Quaternion=one(Quaternion))

    q1 = pbody.state.q2
    q2 = pbody.state.q2 * Δq
    x2 = pbody.state.x2 + vector_rotate(parent_vertex + Δx, q1) - vector_rotate(child_vertex, q2)

    return set_maximal_configurations!(cbody; x=x2, q=q2)
end

function set_maximal_velocities!(pbody::Node, cbody::Body;
    parent_vertex::AbstractVector=SA[0.0; 0.0; 0.0], 
    child_vertex::AbstractVector=SA[0.0; 0.0; 0.0],
    Δv::AbstractVector=SA[0.0; 0.0; 0.0], 
    Δω::AbstractVector=SA[0.0; 0.0; 0.0])

    x1 = pbody.state.x2
    v1 = pbody.state.v15
    q1 = pbody.state.q2
    ω1 = pbody.state.ω15

    x2 = cbody.state.x2
    q2 = cbody.state.q2

    ω2 = vector_rotate(Δω + ω1, inv(q2) * q1)
    ω1w = vector_rotate(ω1, q1)
    ω2w = vector_rotate(ω2, q2)
    Δvw = vector_rotate(Δv, q1)
    cApB_w = (x2 + vector_rotate(child_vertex, q2)) - x1
    pBcB_w = - vector_rotate(child_vertex, q2)
    v2 = copy(v1)
    v2 += skew(ω1w) * cApB_w
    v2 += skew(ω2w) * pBcB_w
    v2 += Δvw

    return set_maximal_velocities!(cbody; v = v2, ω = ω2)
end

"""
    set_external_force!(body; force, torque, vertex)

    applies an external force on a body

    body: Body 
    force: force in body frame
    torque: torque in local frame
    vertex: point where force is applied in local frame
"""
function set_external_force!(body::Body; force=zeros(3), torque=zeros(3), vertex=zeros(3))
    body.state.Fext = vector_rotate(force, body.state.q2)
    body.state.τext = torque + cross(vertex, force)

    return
end

"""
    add_external_force!(body; force, torque, vertex)

    adds an additional external force on a body

    body: Body 
    force: force in body frame
    torque: torque in local frame
"""
function add_external_force!(body::Body; force=zeros(3), torque=zeros(3), vertex=zeros(3))
    body.state.Fext += vector_rotate(force, body.state.q2)
    body.state.τext += torque + cross(vertex, force)

    return
end

function clear_external_force!(body::Body{T}) where T
    body.state.Fext = szeros(T,3)
    body.state.τext = szeros(T,3)

    return
end
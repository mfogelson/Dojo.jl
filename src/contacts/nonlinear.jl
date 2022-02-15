mutable struct NonlinearContact{T,N} <: Contact{T,N}
    friction_coefficient::T
    surface_projector::SMatrix{2,3,T,6}
    surface_normal_projector::Adjoint{T,SVector{3,T}} # inverse matrix
    contact_point::SVector{3,T}
    offset::SVector{3,T}

    function NonlinearContact(body::Body{T}, normal::AbstractVector, friction_coefficient; contact_point = szeros(T, 3), offset::AbstractVector = szeros(T, 3)) where T
        V1, V2, V3 = orthogonalcols(normal) # gives two plane vectors and the original normal axis
        A = [V1 V2 V3]
        Ainv = inv(A)
        surface_normal_projector = Ainv[3,SA[1; 2; 3]]'
        surface_projector = SA{T}[
            1 0 0
            0 1 0
        ]
        new{T,8}(friction_coefficient, surface_projector, surface_normal_projector, contact_point, offset)
    end
end

function constraint(mechanism, contact::ContactConstraint{T,N,Nc,Cs}) where {T,N,Nc,Cs<:NonlinearContact{T,N}}
    model = contact.model
    body = get_body(mechanism, contact.parent_id)
    x2, v25, q2, ϕ25 = current_configuration_velocity(body.state)
    x3, q3 = next_configuration(body.state, mechanism.timestep)

    constraint(model, contact.impulses_dual[2], contact.impulses[2], x3, q3, v25, ϕ25)
end

function constraint(model::NonlinearContact, s::AbstractVector{T}, γ::AbstractVector{T},
        x3::AbstractVector{T}, q3::UnitQuaternion{T}, v25::AbstractVector{T},
        ϕ25::AbstractVector{T}) where T

    # transforms the velocities of the origin of the link into velocities
    vp = v25 + skew(vrotate(ϕ25, q3)) * (vrotate(model.contact_point, q3) - model.offset)
    SVector{4,T}(
        model.surface_normal_projector * (x3 + vrotate(model.contact_point, q3) - model.offset) - s[1],
        model.friction_coefficient * γ[1] - γ[2],
        (model.surface_projector * vp - s[@SVector [3,4]])...)
end

@inline function constraint_jacobian_velocity(model::NonlinearContact{T}, x3::AbstractVector{T}, q3::UnitQuaternion{T},
    x2::AbstractVector{T}, v25::AbstractVector{T}, q2::UnitQuaternion{T}, ϕ25::AbstractVector{T}, λ, timestep::T) where T
    V = [model.surface_normal_projector * timestep;
        szeros(1,3);
        model.surface_projector]
    # Ω = FiniteDiff.finite_difference_jacobian(ϕ25 -> g(model, s, γ, x2+timestep*v25, next_orientation(q2,ϕ25,timestep), v25, ϕ25), ϕ25)
    ∂v∂q3 = skew(vrotate(ϕ25, q3)) * ∂vrotate∂q(model.contact_point, q3)
    ∂v∂q3 += skew(model.offset - vrotate(model.contact_point, q3)) * ∂vrotate∂q(ϕ25, q3)
    ∂v∂ϕ25 = skew(model.offset - vrotate(model.contact_point, q3)) * ∂vrotate∂p(ϕ25, q3)
    Ω = [model.surface_normal_projector * ∂vrotate∂q(model.contact_point, q3) * rotational_integrator_jacobian_velocity(q2, ϕ25, timestep);
        szeros(1,3);
        model.surface_projector * (∂v∂ϕ25 + ∂v∂q3 * rotational_integrator_jacobian_velocity(q2, ϕ25, timestep))]
    return [V Ω]
end

@inline function constraint_jacobian_configuration(model::NonlinearContact{T}, x3::AbstractVector{T}, q3::UnitQuaternion{T},
    x2::AbstractVector{T}, v25::AbstractVector{T}, q2::UnitQuaternion{T}, ϕ25::AbstractVector{T}, λ, timestep::T) where T
    X = [model.surface_normal_projector;
        szeros(1,3);
        szeros(2,3)]
    # Ω = FiniteDiff.finite_difference_jacobian(ϕ25 -> g(model, s, γ, x2+timestep*v25, next_orientation(q2,ϕ25,timestep), v25, ϕ25), ϕ25)
    ∂v∂q3 = skew(vrotate(ϕ25, q3)) * ∂vrotate∂q(model.contact_point, q3)
    ∂v∂q3 += skew(model.offset - vrotate(model.contact_point, q3)) * ∂vrotate∂q(ϕ25, q3)
    Q = [model.surface_normal_projector * ∂vrotate∂q(model.contact_point, q3);
        szeros(1,4);
        model.surface_projector * ∂v∂q3]
    return [X Q]
end

@inline function force_mapping(model::NonlinearContact, x::AbstractVector, q::UnitQuaternion)
    X = [model.surface_normal_projector;
         szeros(1,3);
         model.surface_projector]
    return X
end

@inline function set_matrix_vector_entries!(mechanism::Mechanism, matrix_entry::Entry, vector_entry::Entry,
    contact::ContactConstraint{T,N,Nc,Cs,N½}) where {T,N,Nc,Cs<:NonlinearContact{T,N},N½}
    # ∇impulses[impulses .* impulses - μ; g - s] = [diag(impulses); -diag(0,1,1)]
    # ∇impulses[impulses .* impulses - μ; g - s] = [diag(impulses); -diag(1,0,0)]
    # (friction_coefficient γ - ψ) dependent of ψ = impulses[2][1:1]
    # B(z) * zdot - sβ dependent of sβ = impulses[2][2:end]
    friction_coefficient = contact.model.friction_coefficient
    γ = contact.impulses[2] + 1e-10*neutral_vector(contact.model) # TODO need to check this is legit
    s = contact.impulses_dual[2] + 1e-10*neutral_vector(contact.model) # TODO need to check this is legit

    # ∇s = [contact.impulses[2][1] szeros(1,3); szeros(3,1) cone_product_jacobian(contact.impulses[2][2:4]); Diagonal([-1, 0, -1, -1])]
    ∇s1 = [γ[SA[1]]; szeros(T,3)]'
    ∇s2 = [szeros(T,3,1) cone_product_jacobian(γ[SA[2,3,4]])]
    ∇s3 = Diagonal(SVector{4,T}(-1, 0, -1, -1))
    ∇s = [∇s1; ∇s2; ∇s3]

    # ∇γ = [contact.impulses_dual[2][1] szeros(1,3); szeros(3,1) cone_product_jacobian(contact.impulses_dual[2][2:4]); szeros(1,4); friction_coefficient -1 0 0; szeros(2,4)]
    ∇γ1 = [s[SA[1]]; szeros(T,3)]'
    ∇γ2 = [szeros(T,3,1) cone_product_jacobian(s[SA[2,3,4]])]
    ∇γ3 = SA[0   0 0 0;
             friction_coefficient -1 0 0;
             0   0 0 0;
             0   0 0 0;]
    ∇γ = [∇γ1; ∇γ2; ∇γ3]

    # matrix_entry.value = [[contact.impulses[2][1] szeros(1,3); szeros(3,1) cone_product_jacobian(contact.impulses[2][2:4]); Diagonal([-1, 0, -1, -1])] [contact.impulses_dual[2][1] szeros(1,3); szeros(3,1) cone_product_jacobian(contact.impulses_dual[2][2:4]); szeros(1,4); friction_coefficient -1 0 0; szeros(2,4)]]
    matrix_entry.value = [∇s ∇γ]

    # [-impulses .* impulses + μ; -g + s]
    vector_entry.value = vcat(-complementarityμ(mechanism, contact), -constraint(mechanism, contact))
    return
end

function complementarity(mechanism, contact::ContactConstraint{T,N,Nc,Cs,N½};
        scaling::Bool = false) where {T,N,Nc,Cs<:NonlinearContact{T,N},N½}
    γ = contact.impulses[2]
    s = contact.impulses_dual[2]
    return vcat(γ[1] * s[1], cone_product(γ[@SVector [2,3,4]], s[@SVector [2,3,4]]))
end

neutral_vector(model::NonlinearContact{T,N}) where {T,N} = [sones(T, 2); szeros(T, Int(N/2) -2)]

cone_degree(model::NonlinearContact) = 2
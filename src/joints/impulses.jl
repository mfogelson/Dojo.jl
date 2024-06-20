################################################################################
# Impulse Transform
################################################################################
function impulse_transform(relative::Symbol, joint::Joint, xa::AbstractVector,
        qa::Quaternion, xb::AbstractVector, qb::Quaternion)
    X, Q = displacement_jacobian_configuration(relative, joint, xa, qa, xb, qb, attjac=true)
    Diagonal([sones(3); 0.5 * sones(3)]) * transpose([X Q]) #TODO: 0.5 Q
end

################################################################################
# Derivatives
################################################################################
function impulse_map_jacobian(relative::Symbol, jacobian::Symbol, joint::Joint, pbody::Node{T}, cbody::Node{T}, λ) where T
    # ∂(G*λ)/∂(x,q)
    p = impulse_projector(joint) * λ
    impulse_transform_jacobian(relative, jacobian,
        joint,
        current_configuration(pbody.state)...,
        current_configuration(cbody.state)...,
        p)
end

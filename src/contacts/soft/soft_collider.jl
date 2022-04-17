################################################################################
# ColliderOptions
################################################################################
@with_kw mutable struct ColliderOptions{T}
    impact_damper::T=3e5
    impact_spring::T=3e4
    sliding_drag::T=0.1
    sliding_friction::T=0.2
    rolling_drag::T=0.0
    rolling_friction::T=0.01
    coulomb_smoothing::T=3e1
    coulomb_regularizer::T=1e-3
end

abstract type Collider{T} end
mutable struct SoftCollider{T,N} <: Collider{T}
    mass::T
    inertia::AbstractMatrix{T}
    center_of_mass::AbstractVector{T}
    particles::Vector{SVector{3,T}}
    densities::Vector{T}
    density_gradients::Vector{SVector{3,T}}
    weights::Vector{T} # contribution to collision force F = spring_constant * collision_weight
    weight_gradients::Vector{SVector{3,T}}
    nerf_object::Any
    options::ColliderOptions{T}
end

function SoftCollider(nerf_object; N=1000, density_scale=0.1, opts=ColliderOptions(), T=Float64)
    mass, inertia, center_of_mass = OSFLoader.inertia_properties(nerf_object, density_scale=density_scale)
    particles, densities, density_gradients = OSFLoader.sample_soft(nerf_object, N, particle_noise=0.005)
    weights = densities ./ sum(densities) * mass
    weight_gradients = density_gradients ./ sum(densities) * mass
    return SoftCollider{T,N}(
        mass, inertia, center_of_mass,
        particles, densities, density_gradients,
        weights, weight_gradients, nerf_object,
        opts)
end

function SoftCollider(mass, inertia, center_of_mass, density_fct::Function, gradient_fct::Function;
        N=1000, opts=ColliderOptions(), T=Float64, particle_noise=0.005)

    particles, densities, density_gradients = OSFLoader.sample_soft(density_fct, gradient_fct, N, particle_noise=particle_noise)
    weights = densities ./ sum(densities) * mass
    weight_gradients = density_gradients ./ sum(densities) * mass
    return SoftCollider{T,N}(
        mass, inertia, center_of_mass,
        particles, densities, density_gradients,
        weights, weight_gradients, nothing,
        opts)
end
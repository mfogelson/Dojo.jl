using Test
using BenchmarkTools

using LinearAlgebra
using Random
using SparseArrays
using StaticArrays
using FiniteDiff
using ForwardDiff
using Rotations

using Dojo
Pkg.activate("./DojoEnvironments")
using DojoEnvironments
Pkg.activate(".")
global REG = 1.0e-10

@testset "Integrator"                    verbose=true begin include("integrator.jl") end
@testset "Minimal Coordinates"           verbose=true begin include("minimal.jl") end
@testset "Modified Rodrigues Parameters" verbose=true begin include("mrp.jl") end
@testset "Damper"                        verbose=true begin include("damper.jl") end
@testset "Jacobian (Solution Matrix)"    verbose=true begin include("jacobian.jl") end
@testset "Data"                          verbose=true begin include("data.jl") end
@testset "Collisions"                    verbose=true begin include("collisions.jl") end
@testset "Momentum"                      verbose=true begin include("momentum.jl") end
@testset "Energy"                        verbose=true begin include("energy.jl") end
@testset "Behavior"                      verbose=true begin include("behaviors.jl") end
@testset "Joint Limits"                  verbose=true begin include("joint_limits.jl") end
@testset "Impulse Map"                   verbose=true begin include("impulse_map.jl") end
@testset "Bodies"                        verbose=true begin include("bodies.jl") end
@testset "Mechanism (Miscellaneous)"     verbose=true begin include("mechanism.jl") end
@testset "Simulate"                      verbose=true begin include("simulate.jl") end
@testset "Visuals"                       verbose=true begin include("visuals.jl") end
@testset "Utilities"                     verbose=true begin include("utilities.jl") end

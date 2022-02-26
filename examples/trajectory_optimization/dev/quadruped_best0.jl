# Utils
function module_dir()
    return joinpath(@__DIR__, "..", "..", "..")
end

# Activate package
using Pkg
Pkg.activate(module_dir())

using MeshCat
# Open visualizer
vis = Visualizer()
open(vis)

# Include new files
include(joinpath(module_dir(), "examples", "loader.jl"))
include(joinpath(module_dir(), "src", "optional_components", "trajopt_utils.jl"))

using IterativeLQR

# System
gravity = -9.81
timestep = 0.05
mech = get_mechanism(:quadruped, timestep=timestep, gravity=gravity, friction_coefficient = 1.5, damper = 10.0, spring = 0.0)
initialize!(mech, :quadruped, tran = [0,0,0.], v = [0.5,0,0.])

@elapsed storage = simulate!(mech, 0.05, record = true, solver = :mehrotra!, verbose = false)
visualize(mech, storage, vis = vis)

T = 20
n = minimal_dimension(mech)
m = 12
d = 0
xref = quadruped_trajectory(mech, r = 0.10, z = 0.29; N = Int(T/2), Ncycles = 1)
zref = [minimal_to_maximal(mech, x) for x in xref]
storage = generate_storage(mech, zref)
visualize(mech, storage, vis = vis)
zref = [maximal_to_minimal(mech, z) for z in zref]

z1 = zref[1]
visualize_maximal(mech, minimal_to_maximal(mech, z1), vis)

function gravity_compensation(mechanism::Mechanism)
    # only works with revolute joints for now
    nu = input_dimension(mechanism)
    u = zeros(nu)
    off  = 0
    for joint in mechanism.joints
        nu = input_dimension(joint)
        if joint.parent_id != nothing
            body = get_body(mechanism, joint.parent_id)
            rot = joint.rotational
            A = Matrix(nullspace_mask(rot))
            input = spring_impulses(mechanism, joint, body)
            F = input[1:3]
            τ = input[4:6]
            u[off .+ (1:nu)] = -A * τ
        else
            @warn "need to treat the joint to origin"
        end
        off += nu
    end
    return u
end

mech = get_mechanism(:quadruped, timestep=timestep, gravity=gravity, friction_coefficient = 1.5, damper = 1000.0, spring = 30.0)
initialize!(mech, :quadruped)
@elapsed storage = simulate!(mech, 5.0, record = true, solver = :mehrotra!, verbose = false)
visualize(mech, storage, vis = vis)
ugc = gravity_compensation(mech)

mech = get_mechanism(:quadruped, timestep=timestep, gravity=gravity, friction_coefficient = 1.5, damper = 5.0, spring = 0.0)

u_control = ugc[6 .+ (1:12)]
u_mask = [zeros(12,6) I(m)]

z = [copy(z1)]
for t = 1:5
    znext = maximal_to_minimal(mech, step!(mech, minimal_to_maximal(mech, z[end]), u_mask'*u_control))
    push!(z, znext)
end
storage = generate_storage(mech, [minimal_to_maximal(mech, zi) for zi in z])
visualize(mech, storage, vis = vis)


# Model
function fd(y, x, u, w)
	z = step!(mech, minimal_to_maximal(mech, x), u_mask'*u, ϵ = 3e-4, btol = 3e-4, undercut = 1.5, verbose = false)
	y .= copy(maximal_to_minimal(mech, z))
end

function fdx(fx, x, u, w)
	fx .= copy(get_minimal_gradients(mech, minimal_to_maximal(mech, x), u_mask'*u, ϵ = 3e-4, btol = 3e-4, undercut = 1.5, verbose = false)[1])
end

function fdu(fu, x, u, w)
	∇u = copy(get_minimal_gradients(mech, minimal_to_maximal(mech, x), u_mask'*u, ϵ = 3e-4, btol = 3e-4, undercut = 1.5, verbose = false)[2])
	fu .= ∇u * u_mask'
end


# Time
h = mech.timestep
dyn = Dynamics(fd, fdx, fdu, n, n, m, d)
model = [dyn for t = 1:T-1]


# Initial conditions, controls, disturbances
ū = [u_control for t = 1:T-1]
w = [zeros(d) for t = 1:T-1]

# Rollout


x̄ = rollout(model, z1, ū, w)
# step!(model.mech, x, u_mask'*u_control, ϵ = 1e-6, btol = 1e-6, undercut = 1.5, verbose = false)
# getGradients!(model.mech, x, u_mask'*u_control, ϵ = 1e-6, btol = 1e-3, undercut = 1.5, verbose = false)
storage = generate_storage(mech, [minimal_to_maximal(mech, x) for x in x̄])
visualize(mech, storage; vis = vis)

# Objective
# qt1 = [0.1; 0.1; 1.0; 0.001 * ones(3); 0.01 * ones(4); 0.01 * ones(3)]
# qt2 = [0.1; 0.1; 1.0; 0.001 * ones(3); 0.01 * ones(4); 0.01 * ones(3)]
# body_scale = [1; 0.1ones(12)]
# qt = vcat([body_scale[i] * [0.1 * ones(3); 0.001 * ones(3); 0.1 * ones(4); 0.01 * ones(3)] for i = 1:Nb]...)
qt = [0.3; 0.05; 0.05; 0.01 * ones(3); 0.01 * ones(3); 0.01 * ones(3); fill([0.2, 0.001], 12)...]

# ot1 = (x, u, w) -> transpose(x - zM) * Diagonal(timestep * qt) * (x - zM) + transpose(u) * Diagonal(timestep * 0.01 * ones(m)) * u
# ot2 = (x, u, w) -> transpose(x - zT) * Diagonal(timestep * qt) * (x - zT) + transpose(u) * Diagonal(timestep * 0.01 * ones(m)) * u
# oT = (x, u, w) -> transpose(x - zT) * Diagonal(timestep * qt) * (x - zT)
ots = [(x, u, w) -> transpose(x - zref[t]) * Diagonal(timestep * qt) * (x - zref[t]) + transpose(u) * Diagonal(timestep * 0.01 * ones(m)) * u for t = 1:T-1]
oT = (x, u, w) -> transpose(x - zref[end]) * Diagonal(timestep * qt) * (x - zref[end])

# ct1 = Cost(ot1, n, m, d)
# ct2 = Cost(ot2, n, m, d)
# cT = Cost(oT, n, 0, 0)
cts = Cost.(ots, n, m, d)
cT = Cost(oT, n, 0, 0)
# obj = [[ct1 for t = 1:10]..., [ct2 for t = 1:10]..., cT]
obj = [cts..., cT]

# Constraints
function goal(x, u, w)
	# Δ = x - zT
    Δ = x - zref[end]
    return Δ[collect(1:3)]
end

cont = Constraint()
conT = Constraint(goal, n, 0)
cons = [[cont for t = 1:T-1]..., conT]

prob = problem_data(model, obj, cons)
initialize_controls!(prob, ū)
initialize_states!(prob, x̄)

# Solve
IterativeLQR.constrained_ilqr_solve!(prob,
    verbose = true,
	linesearch=:armijo,
    α_min=1.0e-5,
    obj_tol=1.0e-3,
    grad_tol=1.0e-3,
    max_iter=100,
    max_al_iter=5,
    ρ_init=1.0,
    ρ_scale=10.0)

x_sol, u_sol = get_trajectory(prob)
storage = generate_storage(mech, [minimal_to_maximal(mech, x) for x in x_sol])
visualize(mech, storage, vis = vis)
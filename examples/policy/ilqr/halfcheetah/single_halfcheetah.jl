using Pkg
Pkg.activate(joinpath(module_dir(), "examples"))

using Dojo
using IterativeLQR
using RoboDojo
using Plots
using Symbolics
using BenchmarkTools
using LinearAlgebra
using FiniteDiff

const iLQR = IterativeLQR
const RD = RoboDojo

include("../methods.jl")

vis = Visualizer()
open(vis)


include("../../robodojo/halfcheetah/model.jl")
include("../../robodojo/halfcheetah/visuals.jl")
include("../../robodojo/dynamics.jl")

RoboDojo.RESIDUAL_EXPR
force_codegen = true
# force_codegen = false
robot = halfhyena
include("../../robodojo/codegen.jl")
RoboDojo.RESIDUAL_EXPR




################################################################################
# Simulation
################################################################################
# ## Initial conditions
q1 = nominal_configuration(halfhyena)
v1 = zeros(halfhyena.nq)

# ## Time
h = 0.03
timestep = h
T = 500

# ## Simulator
s = Simulator(halfhyena, T, h=h)
s.ip.opts.r_tol = 1e-5
s.ip.opts.κ_tol = 3e-2
s.ip.opts.undercut = Inf
# ## Simulate
RD.simulate!(s, q1, v1)
# ## Visualize
RD.visualize!(vis, s)


################################################################################
# Dynamics Model
################################################################################
dynamics_model = Simulator(halfhyena, 1, h=h)
dynamics_model.ip.opts.r_tol = 1e-5
dynamics_model.ip.opts.κ_tol = 3e-2
dynamics_model.ip.opts.undercut = 5.0

nq = dynamics_model.model.nq
nx = 2nq
nu = dynamics_model.model.nu
nw = dynamics_model.model.nw


################################################################################
# iLQR
################################################################################
x_hist = [RD.nominal_state(dynamics_model.model)]
for i = 1:75
    y = zeros(nx)
    RD.dynamics(dynamics_model, y, x_hist[end], [0;0;0;0*ones(6)], zeros(nw))
    push!(x_hist, y)
end
plot(hcat(x_hist...)'[:,1:3])

s = Simulator(halfhyena, 100-1, h=h)
for i = 1:75
    q = x_hist[i][1:nq]
    v = x_hist[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)


# ## initialization
x1 = deepcopy(x_hist[1])
xT = deepcopy(x_hist[end])
# xT[1] += 0.30
xT[nq+1] += 3.0
RD.set_robot!(vis, dynamics_model.model, x1)
RD.set_robot!(vis, dynamics_model.model, xT)

u_hover = [0; 0; zeros(nu-2)]

# ## (1-layer) multi-layer perceptron policy
l_input = nx
l1 = 6
l2 = nu
nθ = l1 * l_input + l2 * l1

function policy(θ, x, goal)
    shift = 0
    # input
    input = x - goal

    # layer 1
    W1 = reshape(θ[shift .+ (1:(l1 * l_input))], l1, l_input)
    z1 = W1 * input
    o1 = tanh.(z1)
    shift += l1 * l_input

    # layer 2
    W2 = reshape(θ[shift .+ (1:(l2 * l1))], l2, l1)
    z2 = W2 * o1

    o2 = z2
    return o2
end

# ## horizon
T = 31

# ## model
h = timestep

function f1(y, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = u[nu .+ (1:nθ)]
    RD.dynamics(dynamics_model, view(y, 1:nx), x_di, u_ctrl, w)
    @views y[nx .+ (1:nθ)] .= θ
    return nothing
end

function f1x(dx, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = u[nu .+ (1:nθ)]
    dx .= 0.0
    RD.dynamics_jacobian_state(dynamics_model, view(dx, 1:nx, 1:nx), x_di, u_ctrl, w)
    return nothing
end

function f1u(du, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = u[nu .+ (1:nθ)]
    du .= 0.0
    RD.dynamics_jacobian_input(dynamics_model, view(du, 1:nx, 1:nu), x_di, u_ctrl, w)
    @views du[nx .+ (1:nθ), nu .+ (1:nθ)] .= I(nθ)
    return nothing
end

function ft(y, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = x[nx .+ (1:nθ)]
    RD.dynamics(dynamics_model, view(y, 1:nx), x_di, u_ctrl, w)
    @views y[nx .+ (1:nθ)] .= θ
    return nothing
end

function ftx(dx, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = x[nx .+ (1:nθ)]
    dx .= 0.0
    RD.dynamics_jacobian_state(dynamics_model, view(dx, 1:nx, 1:nx), x_di, u_ctrl, w)
    @views dx[nx .+ (1:nθ), nx .+ (1:nθ)] .= I(nθ)
    return nothing
end

function ftu(du, x, u, w)
    @views u_ctrl = u[1:nu]
    @views x_di = x[1:nx]
    @views θ = x[nx .+ (1:nθ)]
    RD.dynamics_jacobian_input(dynamics_model, view(du, 1:nx, 1:nu), x_di, u_ctrl, w)
    return nothing
end


# user-provided dynamics and gradients
dyn1 = iLQR.Dynamics(f1, f1x, f1u, nx + nθ, nx, nu + nθ)
dynt = iLQR.Dynamics(ft, ftx, ftu, nx + nθ, nx + nθ, nu)

dyn = [dyn1, [dynt for t = 2:T-1]...]

# ## objective
function o1(x, u, w)
    J = 0.0
    q = 1e-2 * [1e-6; ones(nq-1); 1e+0; ones(nq-1)]
    r = 1e-0 * ones(nu)
    ex = x - xT
    eu = u[1:nu] - u_hover
    J += 0.5 * transpose(ex) * Diagonal(q) * ex
    J += 0.5 * transpose(eu) * Diagonal(r) * eu
    J += 1e-1 * dot(u[nu .+ (1:nθ)], u[nu .+ (1:nθ)])
    return J
end

function ot(x, u, w)
    J = 0.0
    q = 1e-2 * [1e-6; ones(nq-1); 1e+0; ones(nq-1)]
    r = 1e-0 * ones(nu)
    ex = x[1:nx] - xT
    eu = u[1:nu] - u_hover
    J += 0.5 * transpose(ex) * Diagonal(q) * ex
    J += 0.5 * transpose(eu) * Diagonal(r) * eu
    J += 1e-1 * dot(x[nx .+ (1:nθ)], x[nx .+ (1:nθ)])
    return J
end

function oT(x, u, w)
    J = 0.0
    return J
end

c1 = iLQR.Cost(o1, nx, nu + nθ)
ct = iLQR.Cost(ot, nx + nθ, nu)
cT = iLQR.Cost(oT, nx + nθ, 0)
obj = [c1, [ct for t = 2:(T - 1)]..., cT]


# ## constraints
ul = -1.0 * [1e0*ones(3); 1e3ones(nu-3)]
uu = +1.0 * [1e0*ones(3); 1e3ones(nu-3)]

function con1(x, u, w)
    θ = u[nu .+ (1:nθ)]
    [
        ul - u[1:nu];
        u[1:nu] - uu;
        1.0e-2 * (u[1:nu] - policy(θ, x[1:nx], xT));
    ]
end

function cont(x, u, w)
    θ = x[nx .+ (1:nθ)]
    [
        ul - u[1:nu];
        u[1:nu] - uu;
        1.0e-2 * (u[1:nu] - policy(θ, x[1:nx], xT))
    ]
end

function goal(x, u, w)
    [
        # x[[1,nq+1]] - xT[[1,nq+1]];
        x[nq+1:nq+1] - xT[nq+1:nq+1];
    ]
end
con_policy1 = iLQR.Constraint(con1, nx, nu + nθ, indices_inequality=collect(1:2nu))
con_policyt = iLQR.Constraint(cont, nx + nθ, nu, indices_inequality=collect(1:2nu))

cons = [con_policy1, [con_policyt for t = 2:T-1]..., iLQR.Constraint(goal, nx + nθ, 0)]

# ## problem
opts = iLQR.Options(line_search=:armijo,
    max_iterations=75,
    max_dual_updates=8,
    objective_tolerance=1e-3,
    lagrangian_gradient_tolerance=1e-3,
    constraint_tolerance=1e-3,
    scaling_penalty=10.0,
    max_penalty=1e7,
    verbose=true)

p = iLQR.Solver(dyn, obj, cons, options=opts)

# ## initialize
θ0 = 1.0 * randn(nθ)
u_guess = [t == 1 ? [u_hover; θ0] : u_hover for t = 1:T-1]
x_guess = iLQR.rollout(dyn, x1, u_guess)
s = Simulator(halfhyena, T-1, h=h)
for i = 1:T
    q = x_guess[i][1:nq]
    v = x_guess[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)

iLQR.initialize_controls!(p, u_guess)
iLQR.initialize_states!(p, x_guess)
dynamics_model.ip.opts.r_tol = 1e-5
dynamics_model.ip.opts.κ_tol = 3e-2
local_continuation_callback!(solver::Solver) = continuation_callback!(solver, dynamics_model)

# ## solve
@time iLQR.constrained_ilqr_solve!(p, augmented_lagrangian_callback! = local_continuation_callback!)


# ## solution
x_sol, u_sol = iLQR.get_trajectory(p)
θ_sol = u_sol[1][nu .+ (1:nθ)]

# ## state
plot(hcat([x[1:nx] for x in x_sol]...)', label="", color=:orange, width=2.0)

# ## control
plot(hcat([u[1:nu] for u in u_sol]..., u_sol[end])', linetype = :steppost)

# ## plot xy
plot([x[1] for x in x_sol], [x[2] for x in x_sol], label="", color=:black, width=2.0)

# ## visualization
s = Simulator(halfhyena, T-1, h=h)
for i = 1:T
    q = x_sol[i][1:nq]
    v = x_sol[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)

# ## simulate policy
x_hist = [x1]
u_hist = [u_hover]

for t = 1:10T
    push!(u_hist, [0;0;0; policy(θ_sol, x_hist[end], xT)[4:end]])
    y = zeros(nx)
    RD.dynamics(dynamics_model, y, x_hist[end], u_hist[end], zeros(nw))
    push!(x_hist, y)
end

s = Simulator(halfhyena, 10T-1, h=h)
for i = 1:10T
    q = x_hist[i][1:nq]
    v = x_hist[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)
set_light!(vis)
set_floor!(vis)

# Dojo.convert_frames_to_video_and_gif("halfhyena_single_regularized_open_loop")
# Dojo.convert_frames_to_video_and_gif("halfhyena_single_regularized_policy")
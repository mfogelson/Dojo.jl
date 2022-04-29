using Pkg
Pkg.activate(joinpath(Dojo.module_dir(), "examples"))

using Dojo
using IterativeLQR
using RoboDojo
using Plots
using Symbolics
using BenchmarkTools
using LinearAlgebra
using FiniteDiff
using StaticArrays

const iLQR = IterativeLQR
const RD = RoboDojo

include("../methods.jl")

vis = Visualizer()
open(vis)


include("../../robodojo/centroidal_quadruped/model.jl")
include("../../robodojo/centroidal_quadruped/visuals.jl")
include("../../robodojo/centroidal_quadruped/simulator.jl")
include("../../robodojo/dynamics.jl")

RoboDojo.RESIDUAL_EXPR
force_codegen = true
# force_codegen = false
robot = centroidal_quadruped
include("../../robodojo/codegen.jl")
RoboDojo.RESIDUAL_EXPR

################################################################################
# Simulation
################################################################################
# ## Initial conditions
q1 = nominal_configuration(RD.centroidal_quadruped) #+ [0; 0; 0.3; 0.1; 0.2; zeros(13)]
v1 = zeros(RD.centroidal_quadruped.nq)

# ## Time
h = 0.02
timestep = h
T = 100

# ## Simulator
s = Simulator(RD.centroidal_quadruped, T, h=h)
s.ip.opts.r_tol = 1e-7
s.ip.opts.κ_tol = 1e-5
s.ip.opts.undercut = Inf
# s.ip.opts.r_tol = 1e-4
# s.ip.opts.κ_tol = 1e-2
# s.ip.opts.undercut = 5.0
# ## Simulate
RD.simulate!(s, q1, v1)
# ## Visualize
RD.visualize!(vis, s)
set_light!(vis)
set_floor!(vis)


################################################################################
# Dynamics Model
################################################################################
dynamics_model = Simulator(RD.centroidal_quadruped, 1, h=h)
# dynamics_model.ip.opts.r_tol = 1e-7
# dynamics_model.ip.opts.κ_tol = 1e-5
# dynamics_model.ip.opts.undercut = 10.0
dynamics_model.ip.opts.r_tol = 1e-5
dynamics_model.ip.opts.κ_tol = 1e-2
dynamics_model.ip.opts.undercut = 5.0

nq = dynamics_model.model.nq
nx = 2nq
nu = dynamics_model.model.nu
nw = dynamics_model.model.nw
nu_infeasible = 6

################################################################################
# Gait design
################################################################################
T = Int(floor(0.65 / h)) + 1
Tm = Int((T + 1) / 2)
s = Simulator(RD.centroidal_quadruped, T, h=h)

gait = trotting_gait(centroidal_quadruped, Tm, timestep=timestep, velocity=0.15)
for x in gait
    RD.set_robot!(vis, centroidal_quadruped, x[1:nq])
    sleep(h)
end

################################################################################
# iLQR
################################################################################
# ## initialization
parameters = deepcopy(gait)
x1 = deepcopy(gait[1])
xT = deepcopy(gait[end])

RD.set_robot!(vis, dynamics_model.model, x1)
RD.set_robot!(vis, dynamics_model.model, xT)

u_hover = [4,0,16, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0.0]

# # ## (2-layer) multi-layer perceptron policy
# l_input = nx-1
# l1 = 6
# l2 = nu - 6
# nθ = l1 * l_input + l2 * l1
#
# function policy(θ, x, goal)
#     shift = 0
#     # input
#     input = (x - goal)[2:end] # policy independent of the x position
#
#     # layer 1
#     W1 = reshape(θ[shift .+ (1:(l1 * l_input))], l1, l_input)
#     z1 = W1 * input
#     o1 = tanh.(z1)
#     shift += l1 * l_input
#
#     # layer 2
#     W2 = reshape(θ[shift .+ (1:(l2 * l1))], l2, l1)
#     z2 = W2 * o1
#
#     o2 = z2
#     return o2
# end

# ## (1-layer) multi-layer perceptron policy
l_input = nx-1 + 1
l_output = nu - 6
nθ = l1 * l_input

function policy(θ, x, goal)
    shift = 0
    # input
    input = [1.0; (x - goal)[2:end]] # policy independent of the x position but uses a bias term

    # layer 1
    W1 = reshape(θ[shift .+ (1:(l_output * l_input))], l_output, l_input)
    z1 = W1 * input
    o1 = tanh.(z1)
    return o1 #z1
end


# ## horizon
T = length(gait)

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
    qbody = [1e-0, 1e-0, 1e+1]
    qfoot = [1e-0, 1e-0, 1e+2]
    q = 1e-0 * [1e-0*qbody; 1e+0*ones(3); [qfoot; qfoot; qfoot; qfoot]]
    v = 1e-0 * [1e-3*ones(3); 1e-2*ones(3); 1e-3*ones(12)]
    r = 1e-2 * [ones(6); [1e-1,1,1e-2]; [1e-1,1,1e-2]; [1e-1,1,1e-2]; [1e-1,1,1e-2]]
    ex = x - w
    eu = u[1:nu] - u_hover
    J += 0.5 * transpose(ex) * Diagonal([q; v]) * ex
    J += 0.5 * transpose(eu) * Diagonal(r) * eu
    J += 1e-1 * dot(u[nu .+ (1:nθ)], u[nu .+ (1:nθ)])
    return J
end

function ot(x, u, w)
    J = 0.0
    qbody = [1e-0, 1e-0, 1e+1]
    qfoot = [1e-0, 1e-0, 1e+2]
    q = 1e-0 * [1e-0*qbody; 1e+0*ones(3); [qfoot; qfoot; qfoot; qfoot]]
    v = 1e-0 * [1e-3*ones(3); 1e-2*ones(3); 1e-3*ones(12)]
    r = 1e-2 * [ones(6); [1e-1,1,1e-2]; [1e-1,1,1e-2]; [1e-1,1,1e-2]; [1e-1,1,1e-2]]
    ex = x[1:nx] - w
    eu = u[1:nu] - u_hover
    J += 0.5 * transpose(ex) * Diagonal([q; v]) * ex
    J += 0.5 * transpose(eu) * Diagonal(r) * eu
    J += 1e-1 * dot(x[nx .+ (1:nθ)], x[nx .+ (1:nθ)])
    return J
end

function oT(x, u, w)
    J = 0.0
    return J
end

c1 = iLQR.Cost(o1, nx, nu + nθ, num_parameter=nx)
ct = iLQR.Cost(ot, nx + nθ, nu, num_parameter=nx)
cT = iLQR.Cost(oT, nx + nθ, 0, num_parameter=nx)
obj = [c1, [ct for t = 2:(T - 1)]..., cT]


# ## constraints
ul = -1.0 * [1e-1*ones(nu_infeasible); 1e3ones(nu-nu_infeasible)]
uu = +1.0 * [1e-1*ones(nu_infeasible); 1e3ones(nu-nu_infeasible)]

function con1(x, u, w)
    θ = u[nu .+ (1:nθ)]
    [
        1e-0 * (ul - u[1:nu]);
        1e-0 * (u[1:nu] - uu);
        1.0e-2 * (u[nu_infeasible+1:nu] - policy(θ, x[1:nx], w));
    ]
end

function cont(x, u, w)
    θ = x[nx .+ (1:nθ)]
    [
        1e-0 * (ul - u[1:nu]);
        1e-0 * (u[1:nu] - uu);
        1.0e-2 * (u[nu_infeasible+1:nu] - policy(θ, x[1:nx], w))
    ]
end

function goal(x, u, w)
    [
        # x[[1,nq+1]] - xT[[1,nq+1]];
        # x[nq+1:nq+1] - xT[nq+1:nq+1];
        1e-0 * (x[1:nq+1] - xT[1:nq+1]);
    ]
end

# con_policy1 = iLQR.Constraint(con1, nx, nu + nθ, num_parameter=nx, indices_inequality=collect(1:2nu))
# con_policyt = iLQR.Constraint(cont, nx + nθ, nu, num_parameter=nx, indices_inequality=collect(1:2nu))
con_policyT = iLQR.Constraint(goal, nx + nθ, 0)

cons = [con_policy1, [con_policyt for t = 2:T-1]..., con_policyT]
# ## problem
opts = iLQR.Options(line_search=:armijo,
    max_iterations=150,
    max_dual_updates=20,
    objective_tolerance=1e-3,
    lagrangian_gradient_tolerance=1e-3,
    constraint_tolerance=1e-3,
    initial_constraint_penalty=1e-3,
    scaling_penalty=2.0,
    max_penalty=1e7,
    verbose=true)

p = iLQR.Solver(dyn, obj, cons, options=opts, parameters=gait)

# ## initialize
θ0 = 1.0 * randn(nθ)
u_guess = [t == 1 ? [u_hover; θ0] : u_hover for t = 1:T-1]
x_guess = iLQR.rollout(dyn, x1, u_guess, parameters)

s = Simulator(RD.centroidal_quadruped, T-1, h=h)
for i = 1:T
    q = x_guess[i][1:nq]
    v = x_guess[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)
# vis = Visualizer()
# open(vis)

iLQR.initialize_controls!(p, u_guess)
iLQR.initialize_states!(p, x_guess)
dynamics_model.ip.opts.r_tol = 1e-6
dynamics_model.ip.opts.κ_tol = 1e-3
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
s = Simulator(RD.centroidal_quadruped, T-1, h=h)
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
    push!(u_hist, [zeros(nu_infeasible); policy(θ_sol, x_hist[end], gait[(t-1)%T+1])])
    y = zeros(nx)
    RD.dynamics(dynamics_model, y, x_hist[end], u_hist[end], zeros(nw))
    push!(x_hist, y)
end

s = Simulator(RD.centroidal_quadruped, 10T-1, h=h)
for i = 1:10T
    q = x_hist[i][1:nq]
    v = x_hist[i][nq .+ (1:nq)]
    RD.set_state!(s, q, v, i)
end
visualize!(vis, s)
set_light!(vis)
set_floor!(vis)

# Dojo.convert_frames_to_video_and_gif("RD.centroidal_quadruped_single_regularized_open_loop")
# Dojo.convert_frames_to_video_and_gif("RD.centroidal_quadruped_single_regularized_policy")

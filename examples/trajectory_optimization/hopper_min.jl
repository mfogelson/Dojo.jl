# PREAMBLE

# PKG_SETUP

# ## setup
using Dojo
using IterativeLQR
using LinearAlgebra

# ## system
dt = 0.05
gravity=-9.81
env = get_environment("raiberthopper",
    mode=:minimal,
    timestep=dt,
    gravity=gravity);

# ## visualizer
open(env.vis)

# ## dimensions
n = env.num_states
m = env.num_inputs

# ## states
z1 = maximal_to_minimal(env.mechanism, raiberthopper_nominal_max())
zM = maximal_to_minimal(env.mechanism, raiberthopper_offset_max(0.5, 0.5, 0.5))
zT = maximal_to_minimal(env.mechanism, raiberthopper_offset_max(0.5, 0.5, 0.0))

# ## nominal control
u_control = [0.0; 0.0; env.mechanism.bodies[1].m * env.mechanism.gravity * env.mechanism.timestep]

# ## horizon
T = 21
Tm = convert(Int, floor((T - 1) / 2))

# ## model
dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> f(y, env, x, u, w),
    (dx, x, u, w) -> fx(dx, env, x, u, w),
    (du, x, u, w) -> fu(du, env, x, u, w),
    n, n, m)

model = [dyn for t = 1:T-1]

# ## rollout
ū = [[0.0; 0.0; env.mechanism.bodies[1].m * env.mechanism.gravity * env.mechanism.timestep + 0.0 * randn(1)[1]] for t = 1:T-1]
x̄ = IterativeLQR.rollout(model, z1, ū)
visualize(env, x̄)

# ## objective
ot1 = (x, u, w) -> 1 * (transpose(x - zM) * Diagonal([1.0 * ones(3); 0.01 * ones(3); 0.1 * ones(3); 0.01 * ones(3); 1.0; 0.01]) * (x - zM) + transpose(u) * Diagonal(1.0e-2 * [1.0; 1.0; 1.0]) * u)
ot2 = (x, u, w) -> 1 * (transpose(x - zT) * Diagonal([1.0 * ones(3); 0.01 * ones(3); 0.1 * ones(3); 0.01 * ones(3); 1.0; 0.01]) * (x - zT) + transpose(u) * Diagonal(1.0e-2 * [1.0; 1.0; 1.0]) * u)
oT = (x, u, w) -> transpose(x - zT) * Diagonal([1.0 * ones(3); 0.01 * ones(3); 0.1 * ones(3); 0.01 * ones(3); 1.0; 0.01]) * (x - zT)

ct1 = Cost(ot1, n, m)
ct2 = Cost(ot2, n, m)
cT = Cost(oT, n, 0)
obj = [[ct1 for t = 1:Tm]..., [ct2 for t = 1:Tm]..., cT]

# ## constraints
function goal(x, u, w)
    Δ = x - zT
    return [Δ[collect(1:3)]; Δ[collect(7:9)]; Δ[13]]
end

cont = IterativeLQR.Constraint()
conT = IterativeLQR.Constraint(goal, n, 0)
cons = [[cont for t = 1:T-1]..., conT]

# ## problem
prob = IterativeLQR.solver(model, obj, cons, 
    opts=Options(
        linesearch=:armijo,
        α_min=1.0e-5,
        obj_tol=1.0e-3,
        grad_tol=1.0e-3,
        con_tol=0.005,
        max_iter=100,
        max_al_iter=10,
        ρ_init=1.0,
        ρ_scale=10.0,
        verbose=true))
IterativeLQR.initialize_controls!(prob, ū)
IterativeLQR.initialize_states!(prob, x̄)

# ## solve
@time IterativeLQR.solve!(prob)

# ## solution
x_sol, u_sol = IterativeLQR.get_trajectory(prob)
@show IterativeLQR.eval_obj(prob.m_data.obj.costs, prob.m_data.x, prob.m_data.u, prob.m_data.w)
@show prob.s_data.iter[1]
@show norm(goal(prob.m_data.x[T], zeros(0), zeros(0)), Inf)

# ## visualize
visualize(env, x_sol)

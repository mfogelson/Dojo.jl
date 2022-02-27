# PREAMBLE

# PKG_SETUP

# ## Setup
using Dojo
using Random
using LinearAlgebra 
include(joinpath(@__DIR__, "algorithms/ars.jl")) # augmented random search

# ## Ant
env = get_environment("ant", mode=:minimal, g=-9.81, timestep=0.05, damper=50.0, spring=25.0, friction_coefficient = 0.5,
    contact=true, contact_body=true)
obs = reset(env)
initialize_ant!(env.mechanism, pos = [1.3,0,0], rot = [0,0,0.])
env.state .= get_minimal_state(env.mechanism)
render(env)

# ## Open visualizer
open(env.vis)

# ## Set up policy
hp = HyperParameters(main_loop_size=100, horizon=150, n_directions=6, b=6, step_size=0.02)
input_size = length(obs)
output_size = length(env.input_previous)
normalizer = Normalizer(input_size)

# ## Training
train_times = Float64[]
rewards = Float64[]
policies = Matrix{Float64}[]
N = 5
for i = 1:N
    ## Reset environment
    env = get_environment("ant", mode=:minimal, g=-9.81, timestep=0.05, damper=50.0, spring=25.0, friction_coefficient = 0.5,
        contact=true, contact_body=true)
    obs = reset(env)

    ## Random policy
    Random.seed!(i)
    hp = HyperParameters(main_loop_size=100, horizon=150, n_directions=6, b=6, step_size=0.02)
    input_size = length(obs)
    output_size = length(env.input_previous)
    normalizer = Normalizer(input_size)
    policy = Policy(input_size, output_size, hp)

    ## Train policy
    train_time = @elapsed train(env, policy, normalizer, hp)

    ## Evaluate policy
    reward = rollout_policy(policy.θ, env, normalizer, hp)

    ## Cache
    push!(train_times, train_time)
    push!(rewards, reward)
    push!(policies, policy.θ)
end

## @save joinpath(@__DIR__, "results/ant_rl.jld2") train_times rewards policies

# ## Training statistics
N_best = 3
max_idx = sortperm(rewards, lt=Base.isgreater)
train_time_best = (train_times[max_idx])[1:N_best]
rewards_best = (rewards[max_idx])[1:N_best]
policies_best = (policies[max_idx])[1:N_best]

@show rewards
@show mean(train_time_best)
@show std(train_time_best)
@show mean(rewards)
@show std(rewards)

# ## Save/Load policy
## θ = policy.θ
## @save joinpath(@__DIR__, "ant_policy.jld2") θ
## @load joinpath(@__DIR__, "ant_policy.jld2") θ

# ## Visualize policy
## traj = display_random_policy(env, hp)
traj = display_policy(env, policy, normalizer, hp)
visualize(env, traj)


using MathOptInterface
const MOI = MathOptInterface
using Ipopt
using Dojo
using LinearAlgebra
using FiniteDiff
function make_body(unit_thickness::Float64, unit_length::Float64, unit_mass::Float64, name::Symbol, color)
    return Cylinder(unit_thickness, unit_length, unit_mass, name=name, color=color)
end

function make_joint(parent::Body, child::Body, rotation_axis::Vector{Float64}, parent_vertex::Vector{Float64}, child_vertex::Vector{Float64}, name::Symbol)
    return JointConstraint(Revolute(parent, child, rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex), name=name)
end

function initialize_mechanism()
    unit_thickness = 0.1
    unit_length = 1.0
    unit_mass = 1.0

    link1 = make_body(unit_thickness, unit_length, unit_mass, :link1, RGBA(1.0, 0, 0))
    link2 = make_body(unit_thickness, unit_length, unit_mass, :link2, RGBA(0, 1.0, 0))
    bodies = [link1, link2]
    joint1 = make_joint(link1, link2, [0, 1.0, 0.], zeros(3), zeros(3), :joint1)
    joints = [joint1]
    origin = Origin()
    mechanism = Mechanism(origin, bodies, joints)
end
# Define the mechanism

# Define the problem structure
mutable struct KinematicChainProblemMOI <: MOI.AbstractNLPEvaluator
    mechanism::Mechanism
    n::Int64
    m::Int64
    joint_dof::Vector{Int64}
    con_jac::Matrix{Float64}
    joint_idx::Vector{UnitRange{Int64}}
    body_idx::Vector{UnitRange{Int64}}
    stepvec::Vector{Float64}
    primal_bounds::Tuple{Vector{Float64}, Vector{Float64}}
    dual_bounds::Tuple{Vector{Float64}, Vector{Float64}}
    
    function KinematicChainProblemMOI(mechanism::Mechanism)
        n = 13 * length(mechanism.bodies)  # assuming 6 DOF per body
        m = length(mechanism.bodies) #sum([Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints])
        joint_dof = [Base.unwrap_unionall(typeof(joint)).parameters[2] for joint in mechanism.joints]
        con_jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, mechanism.bodies, true)
        stepvec = zeros(size(con_jac, 2))
        primal_bounds = (fill(-Inf, n), fill(Inf, n))
        dual_bounds = (fill(0.0, m), fill(0.0, m))
        # new(mechanism, n, m, bounds)
        new(mechanism, n, m, joint_dof, con_jac, joint_idx, body_idx, stepvec, primal_bounds, dual_bounds)
    end
end

# function wrapper_func(mechanism, z)
#     set_maximal_state!(mechanism, z)
#     return Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
# end

# z = get_maximal_state(mechanism)
# z = randn(length(z))
# set_maximal_state!(mechanism, z)
# fo


# using FiniteDiff
# FD_jac = FiniteDiff.finite_difference_jacobian(_z  -> wrapper_func(mechanism, _z), z)

# con_jac, joint_idx, body_idx = initialize_constraint_jacobian(mechanism, mechanism.bodies, false)
# update_constraint_jacobian!(con_jac, mechanism, mechanism.bodies, joint_idx, body_idx, false)

# Define the objective function
function objective_function(problem::KinematicChainProblemMOI, z)
    mechanism = problem.mechanism
    set_maximal_state!(mechanism, z)
    for body in mechanism.bodies
        body.state.q1 = body.state.q1 ./ Dojo.norm(body.state.q1)
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
        # println(Dojo.norm(body.state.q2))
    end
    Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2
    res = Vector(vcat([Dojo.constraint(mechanism, joint) for joint in mechanism.joints]...))
    return 0.5 * dot(res, res)
end

# Define the objective gradient
function objective_gradient!(problem::KinematicChainProblemMOI, grad, z)
    FiniteDiff.finite_difference_gradient!(grad, _z  -> objective_function(problem, _z), z)
    return nothing
end

# Define the constraint vector
function constraint_vector!(problem::KinematicChainProblemMOI, c, z)

    for (i, body) in enumerate(problem.mechanism.bodies)
        # Quaternion normalization constraint
        idx = (7:10).+13*(i-1)
        c[i] = dot(z[idx], z[idx]) - 1.0
    end
    # c[end-2:end] = z[1:3]
    
    return nothing
end

# Define the constraint Jacobian
function constraint_jacobian!(problem::KinematicChainProblemMOI, jac, z)
    jac = reshape(jac, problem.m, problem.n)
    # FiniteDiff.finite_difference_jacobian!(jac, _z  -> constraint_vector!(problem, zeros(problem.m), _z), z)
    for (i, body) in enumerate(problem.mechanism.bodies)
        idx = (7:10).+13*(i-1)
        # println(size(jac))
        # println(2*z[idx])
        jac[i, idx] .= 2*z[idx]
    end
    # jac[end-2:end, 1:3] = Matrix{Float64}(I, 3, 3)

    return nothing
end

# Implement the necessary MOI functions
function MOI.eval_objective(problem::MOI.AbstractNLPEvaluator, z)
    # println("state size $(size(z))")
    return objective_function(problem, z)
end

function MOI.eval_objective_gradient(problem::MOI.AbstractNLPEvaluator, grad, z)
    # println("grad size $(size(grad))")
    return objective_gradient!(problem, grad, z)
end

function MOI.eval_constraint(problem::MOI.AbstractNLPEvaluator, c, z)
    # println("con size $(size(c))")
    return constraint_vector!(problem, c, z)
end

function MOI.eval_constraint_jacobian(problem::MOI.AbstractNLPEvaluator, jac, z)
    # println("Jac size $(size(jac))")
    return constraint_jacobian!(problem, jac, z)
end

function MOI.features_available(problem::MOI.AbstractNLPEvaluator)
    return [:Grad, :Jac]
end

MOI.initialize(problem::MOI.AbstractNLPEvaluator, features) = nothing

function MOI.jacobian_structure(problem::MOI.AbstractNLPEvaluator)
    rows = 1:problem.m
    cols = 1:problem.n
    row_col_pairs = collect(Base.Iterators.product(rows, cols)) 
    return row_col_pairs
end


function solve(prob::KinematicChainProblemMOI; z0 = zeros(problem.n),
    solver=Ipopt.Optimizer(), tol=1.0e-6, c_tol=1.0e-6, max_iter=1000, silent=true)

    # Get Decision Variable Bounds and Constraint Bounds
    z_l, z_u = prob.primal_bounds
    c_l, c_u = prob.dual_bounds

    # A struct holding a pair of lower and upper bounds. -Inf and Inf can be used to indicate no lower or upper bound, respectively.
    nlp_bounds = MOI.NLPBoundsPair.(c_l,c_u)

    # A struct encoding a set of nonlinear constraints 
    block_data = MOI.NLPBlockData(nlp_bounds,prob,true)

    # Solver Options 
    solver.options["max_iter"] = max_iter
    solver.options["tol"] = tol
    solver.options["constr_viol_tol"] = c_tol

    # Add n scalar variables to the model, returning a vector of variable indices.
    n_nlp = prob.n
    z = MOI.add_variables(solver, n_nlp)

    # Set Decision variable constraints
    for i = 1:n_nlp
        # xi = MOI.add_variable(solver)
        MOI.add_constraint(solver, z[i], MOI.LessThan(z_u[i]))
        MOI.add_constraint(solver, z[i], MOI.GreaterThan(z_l[i]))
        MOI.set(solver, MOI.VariablePrimalStart(), z[i], z0[i])
    end

    # Holds the NLPBlockData that represents a set of nonlinear constraints, and optionally a nonlinear objective.
    MOI.set(solver, MOI.NLPBlock(), block_data)

    # A model attribute for the objective sense of the objective function, which must be an OptimizationSense: MIN_SENSE, MAX_SENSE, or FEASIBILITY_SENSE. The default is FEASIBILITY_SENSE.
    MOI.set(solver, MOI.ObjectiveSense(), MOI.MIN_SENSE) # 

    # Not Verbose
    MOI.set(solver, MOI.Silent(), silent)
    # Start the solution procedure.
    MOI.optimize!(solver)


    # Get the solution
    zsol = MOI.get(solver, MOI.VariablePrimal(), z)
    status = MOI.get(solver, MOI.TerminationStatus())
    obj = MOI.get(solver, MOI.ObjectiveValue())

    return zsol, status, obj
end

# mechanism = initialize_mechanism()
path = joinpath(@__DIR__, "/mnt/nvme/home/mitch/.julia/dev/Dojo/DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
mechanism = Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
# mechanism = initialize_mechanism(30)

data = Dict(
    "j2_j1" => Dict("length" => 50.0, "position" => (-4.5067675485638965, 15.636048727421338), "angle" => -38.71463269805005, "label" => :pair06_leg2_bar_j),
    "j5_j1" => Dict("length" => 61.9, "position" => (-5.976053515786479, -22.75758508504058), "angle" => 47.33270859961152, "label" => :pair06_leg2_bar_k),
    "j5_j3" => Dict("length" => 39.3, "position" => (-32.476053515786475, -26.65758508504058), "angle" => 106.32687427036096, "label" => :pair06_leg2_bar_c),
    "j2_j3" => Dict("length" => 41.5, "position" => (-31.006767548563896, 11.736048727421338), "angle" => -109.69561783022645, "label" => :pair06_leg2_bars_b_d_e),
    # "j2_j4" => Dict("length" => 55.8, "position" => (-49.40395023903193, 19.70763383036941), "angle" => -155.51236186405146, "label" => "e"),
    # "j4_j3" => Dict("length" => 40.1, "position" => (-56.39718269046803, 0.17158510294807128), "angle" => -23.42730995038722, "label" => "d"),
    "j5_j6" => Dict("length" => 36.7, "position" => (-43.091810996493976, -36.784050200414455), "angle" => 151.5878748768938, "label" => :pair06_leg2_bars_g_h_i),
    "j4_j6" => Dict("length" => 39.4, "position" => (-67.01294017117553, -9.954880012425807), "angle" => -66.7342627136313, "label" => :pair06_leg2_bar_f),
    # "j5_j7" => Dict("length" => 49.0, "position" => (-35.056108777838844, -68.63605154810219), "angle" => -109.31585688104902, "label" => "i"),
    # "j7_j6" => Dict("length" => 65.7, "position" => (-51.195812742759855, -59.904931578435495), "angle" => 104.15926149980457, "label" => "h")
    "j2_j1" => Dict("length" => 50.0, "position" => (4.5067675485638965, 15.636048727421338), "angle" => -180-38.71463269805005, "label" => :pair06_leg1_bar_j),
    "j5_j1" => Dict("length" => 61.9, "position" => (5.976053515786479, -22.75758508504058), "angle" => 180-47.33270859961152, "label" => :pair06_leg1_bar_k),
    "j5_j3" => Dict("length" => 39.3, "position" => (32.476053515786475, -26.65758508504058), "angle" => 180-106.32687427036096, "label" => :pair06_leg1_bar_c),
    "j2_j3" => Dict("length" => 41.5, "position" => (31.006767548563896, 11.736048727421338), "angle" => -180-109.69561783022645, "label" => :pair06_leg1_bars_b_d_e),
    # "j2_j4" => Dict("length" => 55.8, "position" => (-49.40395023903193, 19.70763383036941), "angle" => -155.51236186405146, "label" => "e"),
    # "j4_j3" => Dict("length" => 40.1, "position" => (-56.39718269046803, 0.17158510294807128), "angle" => -23.42730995038722, "label" => "d"),
    "j5_j6" => Dict("length" => 36.7, "position" => (43.091810996493976, -36.784050200414455), "angle" => 180-151.5878748768938, "label" => :pair06_leg1_bars_g_h_i),
    "j4_j6" => Dict("length" => 39.4, "position" => (67.01294017117553, -9.954880012425807), "angle" => -180-66.7342627136313, "label" => :pair06_leg1_bar_f),
    # "j5_j7" => Dict("length" => 49.0, "position" => (-35.056108777838844, -68.63605154810219), "angle" => -109.31585688104902, "label" => "i"),
    # "j7_j6" => Dict("length" => 65.7, "position" => (-51.195812742759855, -59.904931578435495), "angle" => 104.15926149980457, "label" => "h")
)

for (key, value) in data
    print(value["label"])
    body = get_body(mechanism, Symbol(value["label"]))
    # print(value["position"][1]./100)
    print(value["position"][1] / 100.)
    body.state.x2 = [value["position"][1] / 100., body.state.x2[2],  value["position"][2] / 100.]
    # body.state.x2  =
    body.state.q2 = Quaternion(cos(deg2rad(value["angle"])/2.), 0.0, sin(deg2rad(value["angle"])/2.), 0.0)
end
visualize(mechanism, vis=vis, visualize_floor=false)
# Initialize and solve
problem = KinematicChainProblemMOI(mechanism)
# z0 = rand(problem.n)
# z0[1:3] .= 0.0
z0 = get_maximal_state(mechanism)
solution, status, objective_value = solve(problem, z0=z0)

vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false)

x = get_minimal_state(mechanism)
set_minimal_coordinates!(mechanism, get_joint(mechanism, :pair06_leg1_joint_j_k), [pi/4])


set_maximal_state!(mechanism, solution)
visualize(mechanism, vis=vis, visualize_floor=false)
get_minimal_state(mechanism)
# println("Solution: ", solution)
# println("Status: ", status)
# println("Objective value: ", objective_value)
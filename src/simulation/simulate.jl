"""
    simulate!(mechanism, steps, storage, control!;
        record, verbose, abort_upon_failure, opts)

    simulate a mechanism

    mechanism: Mechanism 
    steps: range of steps to simulate 
    storage: Storage
    control!: Function setting inputs for mechanism
    record: flag for recording simulation to storage
    verbose: flag for printing during simulation 
    abort_upon_failure: flag for terminating simulation is solver fails to meet tolerances
    opts: SolverOptions
"""
function simulate!(mechanism::Mechanism, steps::AbstractUnitRange, storage::Storage,
        control!::Function=(m, k) -> nothing;
        record::Bool=true,
        verbose::Bool=false,
        abort_upon_failure::Bool=false,
        opts=SolverOptions(verbose=verbose))

    initialize_simulation!(mechanism)

    for k = steps
        control!(mechanism, k)
        for joint in mechanism.joints input_impulse!(joint, mechanism) end
        status = mehrotra!(mechanism, opts=opts)
        for body in mechanism.bodies clear_external_force!(body) end
        abort_upon_failure && (status == :failed) && break
        record && save_to_storage!(mechanism, storage, k)
        (k != steps[end]) && (for body in mechanism.bodies update_state!(body, mechanism.timestep) end)
    end

    record ? (return storage) : (return)
end

function simulate!(mechanism::Mechanism{T}, tend::Real, args...;
        record::Bool=true,
        verbose::Bool=false,
        abort_upon_failure::Bool=false,
        opts=SolverOptions(verbose=verbose)) where T

    steps = Base.OneTo(Int64(ceil(tend / mechanism.timestep)))
    record ? (storage = Storage{T}(steps, length(mechanism.bodies))) : (storage = Storage{T}())

    storage = simulate!(mechanism, steps, storage, args...; verbose=verbose,
        record=record, abort_upon_failure=abort_upon_failure, opts=opts)
        
    return storage
end

function initialize_simulation!(mechanism::Mechanism)
    initialize_state!(mechanism)
    for body in mechanism.bodies
        set_velocity_solution!(body)
    end
end

using Dojo
using LinearAlgebra

"""
Create a cylinder body for a link.
"""
function create_link(length, radius, density, color)
    mass = density * π * radius^2 * length
    return Cylinder(radius, length, mass, color=RGBA(color...))
end

"""
Create a generalized mechanism with specified joint types and parameters.
"""
function create_mechanism(
    link_params,
    joint_params,
    joint_vertices;
    use_joint_limits=false,
    joint_limit=1e-5,
    rotation_axis=[0, 1, 0],
    gravity=[0, -9.81, 0],
    timestep=0.001, 
    density=2700.0
)
    origin = Origin{Float64}()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    # Create links
    for (length, radius, color) in link_params
        push!(bodies, create_link(length, radius, density, color))
    end

    # Create joints
    for (i, (joint_param, vertices)) in enumerate(zip(joint_params, joint_vertices))
        joint_type, params = joint_param
        parent, child = params
        parent = parent == :origin ? origin : bodies[parent]
        child = bodies[child]

        joint_kwargs = Dict{Symbol, Any}(
            :parent_vertex => vertices[1],
            :child_vertex => vertices[2],
            :damper => DAMPER_COEFFICIENT
        )
        
        if use_joint_limits && joint_type == PlanarAxis
            joint_kwargs[:tra_joint_limits] = [[-joint_limit, -joint_limit], [joint_limit, joint_limit]]
        end

        push!(joints, JointConstraint(joint_type(parent, child, rotation_axis; joint_kwargs...), name=Symbol("joint$i")))
    end

    return Mechanism(origin, bodies, joints, gravity=gravity, timestep=timestep)
end

"""
Initialize the mechanism's configuration.
"""
function initialize_mechanism!(mechanism)
    freeids = Set(Dojo.getid.(mechanism.bodies))
    z0 = get_maximal_configuration(mechanism, freeids)
    z0 = rand(length(z0))
    set_configuration!(mechanism, freeids, z0)
    for body in mechanism.bodies
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
    end
    newtonIter = 200
    storage = Storage(newtonIter, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=[], newtonIter=newtonIter, lineIter=10, ε=1e-8, debug=true, storage=storage)
    z0_initialized = deepcopy(get_maximal_state(mechanism))
    return z0_initialized
end

"""
Simulate mechanism and calculate energies
"""
function simulate_mechanism(mechanism, simulation_time; control_func=nothing)
    opts = SolverOptions(verbose=true, rtol=1e-8, btol=1e-8, reg=1e-8, max_iter=1000)
    if !isnothing(control_func)
        storage = Dojo.simulate!(mechanism, simulation_time, control_func, record=true, opts=opts)
    else
        storage = Dojo.simulate!(mechanism, simulation_time, record=true, opts=opts)
    end
    ke = kinetic_energy(mechanism, storage)
    pe = potential_energy(mechanism, storage)
    me = mechanical_energy(mechanism, storage)
    
    return storage, ke, pe, me
end

"""
visualize mechanism
"""
function visualize_mechanism(mechanism; storage=nothing, vis=nothing, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.1)
    if isdefined(Main, :vis)
        delete!(vis)
    else
        vis = Visualizer()
    end

    if !isnothing(storage)
        vis = visualize(mechanism, storage; vis=vis, visualize_floor=visualize_floor, show_frame=show_frame, show_joint=show_joint, joint_radius=joint_radius)
    else
        vis = visualize(mechanism; vis=vis, visualize_floor=visualize_floor, show_frame=show_frame, show_joint=show_joint, joint_radius=joint_radius)
    end

    return vis
end


"""
Plot energy comparison for multiple mechanisms
"""
function plot_energy_comparison(mechanisms, kes, pes, mes, slop_values)
    plot_me = plot(title="Mechanical Energy Comparison", xlabel="Time (s)", ylabel="Energy (J)")
    plot_pe = plot(title="Potential Energy Comparison", xlabel="Time (s)", ylabel="Energy (J)")
    plot_ke = plot(title="Kinetic Energy Comparison", xlabel="Time (s)", ylabel="Energy (J)")
    
    colors = [:red, :blue, :green, :purple, :orange, :cyan]
    
    for (i, (mechanism, ke, pe, me, slop, color)) in enumerate(zip(mechanisms, kes, pes, mes, slop_values, colors))
        total_time = 0:mechanism.timestep:length(me)*mechanism.timestep-mechanism.timestep
        
        plot!(plot_me, collect(total_time), me, label="Slop: $slop", color=color)
        plot!(plot_pe, collect(total_time), pe, label="Slop: $slop", color=color)
        plot!(plot_ke, collect(total_time), ke, label="Slop: $slop", color=color)
    end
    
    return plot(plot_me, plot_pe, plot_ke, layout=(3,1), size=(1000, 1200))
end

"""
Plot body positions for multiple mechanisms
"""
function plot_body_positions_comparison(storages, slop_values)
    num_bodies = length(storages[1].x)
    num_steps = length(storages[1].x[1])
    
    plot = plot3d(
        xlabel="X", ylabel="Y", zlabel="Z",
        title="3D Position of Bodies (Slop Comparison)",
        legend=:outerright,
        size=(1000, 800),
        camera=(30, 30)
    )
    
    colors = [:red, :blue, :green, :purple, :orange, :cyan]
    
    for (i, (storage, slop, color)) in enumerate(zip(storages, slop_values, colors))
        positions = [
            [[storage.x[j][k][l] for k in 1:num_steps] for l in 1:3]
            for j in 1:num_bodies
        ]
        
        for (j, pos) in enumerate(positions)
            plot!(pos..., 
                label="Body $j (Slop: $slop)", 
                seriestype=:scatter,
                markersize=2,
                markerstrokewidth=0,
                color=color
            )
        end
    end
    
    return plot
end
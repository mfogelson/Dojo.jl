using Dojo
using LinearAlgebra
using Plots
gr() # Use GR backend
include("/Users/mitchfogelson/.julia/dev/Dojo.jl/ClosedChainEnvironments/src/utils/initialize_constraints_refactor.jl")
# Constants
const CYLINDER_RADIUS = 0.0254  # m
const MATERIAL_DENSITY = 500  # kg/m^3
const ROTATION_AXIS = [0, 1, 0]
const DAMPER_COEFFICIENT = 1.0
const REGULARIZATION = 1e-10

"""
Create a cylinder body for the mechanism.
"""
function create_cylinder(length, color)
    mass = MATERIAL_DENSITY * π * CYLINDER_RADIUS^2 * length
    return Cylinder(CYLINDER_RADIUS, length, mass, color=RGBA(color...))
end

"""
Create a four-bar mechanism.
"""
function create_four_bar_mechanism(use_joint_limits=false, joint_limit=1.25e-5)
    timestep = 0.01
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    # Link lengths
    lengths = [1.4, 1.2, 1.0, 0.4] #[0.1016, 0.3556, 0.3048, 0.254]  # [4, 14, 12, 10] inches
    colors = [(1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)]

    for (i, length) in enumerate(lengths[1:3])
        push!(bodies, create_cylinder(length, colors[i]))
    end

    # Create joints
    joint_types = use_joint_limits ? [Revolute, PlanarAxis, PlanarAxis, PlanarAxis] : [Revolute, Revolute, Revolute, Revolute]
    joint_params = [
        (origin, bodies[1]),
        (bodies[1], bodies[2]),
        (bodies[3], bodies[2]),
        (origin, bodies[3])
    ]
    joint_vertices = [
        ([0, 0, 0], [0, 0, -lengths[1]/2]),
        ([0, 0, lengths[1]/2], [0, 0, -lengths[2]/2]),
        ([0, 0, -lengths[3]/2], [0, 0, lengths[2]/2]),
        ([-lengths[4], 0, 0], [0, 0, lengths[3]/2])
    ]

    for (i, (joint_type, params, vertices)) in enumerate(zip(joint_types, joint_params, joint_vertices))
        joint_kwargs = Dict{Symbol, Any}(
            :parent_vertex => vertices[1],
            :child_vertex => vertices[2]
        )
        
        if i == 1
            joint_kwargs[:damper] = 0.1
        elseif use_joint_limits && i > 1
            joint_kwargs[:tra_joint_limits] = [[-joint_limit, -joint_limit], [joint_limit, joint_limit]]
        end

        push!(joints, JointConstraint(joint_type(params..., ROTATION_AXIS; joint_kwargs...), name=Symbol("joint$i")))
    end

    return Mechanism(origin, bodies, joints, timestep=timestep)
end

"""
Initialize the mechanism's configuration.
"""
function initialize_mechanism!(mechanism)
    freeids = Set(Dojo.getid.(mechanism.bodies))
    z0 = get_maximal_configuration(mechanism, freeids) #.+ 1e-5
    z0 = rand(length(z0))
    set_configuration!(mechanism, freeids, z0)
    for body in mechanism.bodies
        body.state.q2 = body.state.q2 ./ Dojo.norm(body.state.q2)
        body.state.x2 = [body.state.x2[1], body.state.x2[2], 0.0]
    end

    newtonIter = 200
    storage = Storage(newtonIter, length(mechanism.bodies))
    initialize_joint_constraints(mechanism, z0, fixedids=[], newtonIter = newtonIter, lineIter = 10, ε = 1e-8, debug=true, storage=storage)
    z0_revolute = deepcopy(get_maximal_state(mechanism))
    return z0_revolute
end

"""
Control function for the mechanism simulation.
"""
function control!(mechanism, t)
    print_angle(mechanism, mechanism.joints[1])
    if norm(mechanism.bodies[1].state.v15) > 1.5
        return nothing
    end
    set_input!(get_joint(mechanism, :joint1), [-1.0])
end

"""
Print the angle of a specified joint.
"""
function print_angle(mechanism, joint)
    println("Joint Angle: $(Dojo.minimal_coordinates(mechanism, joint))")
end

"""
Simulate the mechanism and visualize the results.
"""
function simulate_and_visualize(mechanism, simulation_time)
    opts = SolverOptions(verbose=true, rtol=1e-6, btol=1e-6, reg=REGULARIZATION, max_iter=50)
    storage = Dojo.simulate!(mechanism, simulation_time, control!, record=true, opts=opts)
    
    vis = Visualizer()
    vis = visualize(mechanism, storage, vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.1)
    
    return storage, vis
end

"""
Plot the 3D positions of the bodies over time and create an animation.
"""
function plot_body_positions(storage, save_plot=false, animate=true)
    num_bodies = length(storage.x)
    num_steps = length(storage.x[1])
    
    positions = [
        [[storage.x[i][j][k] for j in 1:num_steps] for k in 1:3]
        for i in 1:num_bodies
    ]
    
    # Create static 3D plot
    static_plot = plot3d(
        xlabel="X", ylabel="Y", zlabel="Z",
        title="3D Position of Bodies",
        legend=:outerright,
        size=(800, 600),
        camera=(30, 30)
    )
    
    for (i, pos) in enumerate(positions)
        plot!(static_plot, pos..., 
            label="Body $i", 
            seriestype=:scatter,
            markersize=2,
            markerstrokewidth=0
        )
    end

    if save_plot
        savefig(static_plot, "body_positions.png")
    end
    
    # Create animation
    anim = @animate for t in 1:num_steps
        plot_frame = deepcopy(static_plot)
        for (i, pos) in enumerate(positions)
            scatter!(plot_frame, [pos[1][t]], [pos[2][t]], [pos[3][t]], 
                label="", 
                markersize=6, 
                markercolor=:red,
                markerstrokewidth=0
            )
        end
        title!(plot_frame, "3D Position of Bodies (Time step: $t)")
    end every 5  # Update every 5 frames to reduce file size

    if animate
        gif(anim, "body_positions.gif", fps = 30)
    end
    
    return static_plot, anim
end

"""
Simulate mechanism and calculate energies
"""
function simulate_mechanism(mechanism, simulation_time)
    opts = SolverOptions(verbose=false, rtol=1e-6, btol=1e-6, reg=REGULARIZATION, max_iter=50)
    storage = Dojo.simulate!(mechanism, simulation_time, control!, record=true, opts=opts)
    
    ke = kinetic_energy(mechanism, storage)
    pe = potential_energy(mechanism, storage)
    me = mechanical_energy(mechanism, storage)
    
    return storage, ke, pe, me
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

# Main execution
slop_values = [0.0, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1]
mechanisms = []
storages = []
kes = []
pes = []
mes = []

ideal_mech = create_four_bar_mechanism()
z0 = initialize_mechanism!(ideal_mech)
for slop in slop_values
    mechanism = create_four_bar_mechanism(slop != 0.0, slop)

    set_maximal_state!(mechanism, z0)
    
    push!(mechanisms, mechanism)
    
    simulation_time = 500 * mechanism.timestep
    storage, ke, pe, me = simulate_mechanism(mechanism, simulation_time)
    
    push!(storages, storage)
    push!(kes, ke)
    push!(pes, pe)
    push!(mes, me)
end

# Plot body positions comparison
position_plot = plot_body_positions_comparison(storages, slop_values)
display(position_plot)
savefig(position_plot, "body_positions_comparison.png")

# Plot energy comparison
energy_plot = plot_energy_comparison(mechanisms, kes, pes, mes, slop_values)
display(energy_plot)
savefig(energy_plot, "energy_comparison.png")

# Create animation (optional)
anim = @animate for t in 1:length(storages[1].x[1])
    plot_frame = deepcopy(position_plot)
    for (i, (storage, slop, color)) in enumerate(zip(storages, slop_values, [:red, :blue, :green, :purple, :orange, :cyan]))
        for j in 1:length(storage.x)
            scatter!(plot_frame, [storage.x[j][t][1]], [storage.x[j][t][2]], [storage.x[j][t][3]], 
                label="", 
                markersize=6, 
                markercolor=color,
                markerstrokewidth=0
            )
        end
    end
    title!(plot_frame, "3D Position of Bodies (Time step: $t)")
end every 5  # Update every 5 frames to reduce file size

gif(anim, "animated_body_positions_comparison.gif", fps = 30)

delete!(vis)
visualize(mechanisms[end], storages[end], vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.1)

# Main execution
mechanism = create_four_bar_mechanism()
z0 = initialize_mechanism!(mechanism)
mechanism = create_four_bar_mechanism(true)
set_maximal_state!(mechanism, z0)
simulation_time = 500 * mechanism.timestep
storage, vis = simulate_and_visualize(mechanism, simulation_time)
# Usage example (add this to your main execution section)
static_plot, anim = plot_body_positions(storage)

ke0 = kinetic_energy(mechanism, storage)
pe0 = potential_energy(mechanism, storage)
me0 = mechanical_energy(mechanism, storage)

# plot energy
total_time = 0:mechanism.timestep:simulation_time-mechanism.timestep
plot(collect(total_time), me0, title="Mechanical Energy", xlabel="Time (s)", ylabel="Energy (J)", label="Mechanical Energy")
# plot potential energy
plot!(collect(total_time),pe0, title="Potential Energy", xlabel="Time (s)", ylabel="Energy (J)", label="Potential Energy")
# plot kinetic energy
plot!(collect(total_time),ke0, title="Kinetic Energy", xlabel="Time (s)", ylabel="Energy (J)", label="Kinetic Energy")


ke1 = kinetic_energy(mechanism, storage)
pe1 = potential_energy(mechanism, storage)
me1 = mechanical_energy(mechanism, storage)

# plot energy
total_time = 0:mechanism.timestep:simulation_time-mechanism.timestep
plot!(collect(total_time), me1, title="Mechanical Energy slop", xlabel="Time (s)", ylabel="Energy (J)", label="Mechanical Energy")
# plot potential energy
plot!(collect(total_time),pe1, title="Potential Energy slop", xlabel="Time (s)", ylabel="Energy (J)", label="Potential Energy")
# plot kinetic energy
plot!(collect(total_time),ke1, title="Kinetic Energy slop", xlabel="Time (s)", ylabel="Energy (J)", label="Kinetic Energy")
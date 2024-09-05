using Dojo
using LinearAlgebra
using Plots
gr()

# Constants
const CYLINDER_RADIUS = 0.005  # m
const MATERIAL_DENSITY = 2700  # kg/m^3 (aluminum)
const GRAVITY = -9.81  # m/s^2
const DAMPER_COEFFICIENT = 0.1
const REGULARIZATION = 1e-10

"""
Create a cylinder body for the mechanism.
"""
function create_cylinder(length, color)
    mass = MATERIAL_DENSITY * π * CYLINDER_RADIUS^2 * length
    return Cylinder(CYLINDER_RADIUS, length, mass, color=RGBA(color...))
end

"""
Create a Bennett linkage mechanism.
"""
function create_bennett_linkage(use_joint_limits=false)
    timestep = 0.01
    origin = Origin{Float64}()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    # Link lengths
    short_length = 0.05  # 5cm
    long_length = 0.08   # 8cm
    lengths = [short_length, long_length, short_length, long_length]
    colors = [(1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (1.0, 1.0, 0)]

    # Calculate Bennett linkage parameters
    α = acos((long_length^2 - short_length^2) / (long_length^2 + short_length^2))
    β = π - α

    for (i, length) in enumerate(lengths)
        push!(bodies, create_cylinder(length, colors[i]))
    end

    # Create joints
    joint_axes = [
        [0, 1, 0],
        [sin(α), 0, cos(α)],
        [0, -1, 0],
        [-sin(α), 0, cos(α)]
    ]

    joint_positions = [
        [0, 0, 0],
        [short_length/2, 0, 0],
        [(short_length + long_length * cos(α))/2, 0, (long_length * sin(α))/2],
        [long_length/2, 0, 0]
    ]

    for i in 1:4
        parent = i == 1 ? origin : bodies[i-1]
        child = bodies[i]
        axis = joint_axes[i]
        position = joint_positions[i]

        joint = JointConstraint(Revolute(parent, child, axis; 
            parent_vertex=position, 
            child_vertex=[0, 0, -lengths[i]/2],
            damper=DAMPER_COEFFICIENT), 
            name=Symbol("joint$i"))

        if use_joint_limits && i > 1
            joint.constraint.joint_limits = [[-0.1], [0.1]]  # Add small joint limits
        end

        push!(joints, joint)
    end

    mechanism = Mechanism(origin, bodies, joints, gravity=[0, GRAVITY, 0], timestep=timestep)
    return mechanism
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
    set_input!(get_joint(mechanism, :joint1), [0.1 * sin(t)])
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
    
    colors = [:red, :blue, :green, :purple]
    
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
    
    colors = [:red, :blue, :green, :purple]
    
    for (i, (mechanism, ke, pe, me, slop, color)) in enumerate(zip(mechanisms, kes, pes, mes, slop_values, colors))
        total_time = 0:mechanism.timestep:length(me)*mechanism.timestep-mechanism.timestep
        
        plot!(plot_me, collect(total_time), me, label="Slop: $slop", color=color)
        plot!(plot_pe, collect(total_time), pe, label="Slop: $slop", color=color)
        plot!(plot_ke, collect(total_time), ke, label="Slop: $slop", color=color)
    end
    
    return plot(plot_me, plot_pe, plot_ke, layout=(3,1), size=(1000, 1200))
end

# Main execution
slop_values = [0.0]
mechanisms = []
storages = []
kes = []
pes = []
mes = []

ideal_mechanism = create_bennett_linkage()
z0 = initialize_mechanism!(ideal_mechanism)
delete!(vis)

visualize(ideal_mechanism, vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.001)
for slop in slop_values
    mechanism = create_bennett_linkage(slop != 0.0)
    if slop != 0.0
        for joint in mechanism.joints[2:end]
            if hasfield(typeof(joint.constraint), :joint_limits)
                joint.constraint.joint_limits = [[-slop], [slop]]
            end
        end
    end
    
    if isempty(mechanisms)
        z0 = initialize_mechanism!(mechanism)
    else
        set_maximal_state!(mechanism, z0)
    end
    
    push!(mechanisms, mechanism)
    
    simulation_time = 10.0  # Simulate for 10 seconds
    storage, ke, pe, me = simulate_mechanism(mechanism, simulation_time)
    
    push!(storages, storage)
    push!(kes, ke)
    push!(pes, pe)
    push!(mes, me)
end

visualize(mechanisms[1], storages[1]; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.001)

# Plot body positions comparison
position_plot = plot_body_positions_comparison(storages, slop_values)
display(position_plot)
savefig(position_plot, "bennett_positions_comparison.png")

# Plot energy comparison
energy_plot = plot_energy_comparison(mechanisms, kes, pes, mes, slop_values)
display(energy_plot)
savefig(energy_plot, "bennett_energy_comparison.png")

# Create animation
anim = @animate for t in 1:length(storages[1].x[1])
    plot_frame = deepcopy(position_plot)
    for (i, (storage, slop, color)) in enumerate(zip(storages, slop_values, [:red, :blue, :green, :purple]))
        for j in 1:length(storage.x)
            scatter!(plot_frame, [storage.x[j][t][1]], [storage.x[j][t][2]], [storage.x[j][t][3]], 
                label="", 
                markersize=6, 
                markercolor=color,
                markerstrokewidth=0
            )
        end
    end
    title!(plot_frame, "Bennett Linkage - 3D Position (Time step: $t)")
end every 5  # Update every 5 frames to reduce file size

gif(anim, "animated_bennett_positions_comparison.gif", fps = 30)
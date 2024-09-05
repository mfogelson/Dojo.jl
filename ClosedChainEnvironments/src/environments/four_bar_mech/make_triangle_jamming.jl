using Dojo
using LinearAlgebra
using Plots
gr()
include("/Users/mitchfogelson/.julia/dev/Dojo.jl/ClosedChainEnvironments/src/utils/initialize_constraints_refactor.jl")
include("/Users/mitchfogelson/.julia/dev/Dojo.jl/ClosedChainEnvironments/src/utils/mechanism_core.jl")


# Constants
const LINK_LENGTH = 0.1  # 10 cm
const LINK_RADIUS = 0.005  # 5 mm
const MATERIAL_DENSITY = 2700  # kg/m^3 (aluminum)
const GRAVITY = -9.81  # m/s^2
const DAMPER_COEFFICIENT = 0.0
const SLOP_VALUES = [0.0, 1e-4, 1e-3]  # Different slop values to test

# Example usage for triangular linkage
function create_triangular_linkage(use_joint_limits=false, joint_limit=1e-5)
    link_length = 0.1  # 10 cm
    link_radius = 0.005  # 5 mm
    link_params = [
        (link_length, link_radius, (1.0, 0, 0)),
        (link_length, link_radius, (0, 1.0, 0)),
        (link_length, link_radius, (0, 0, 1.0))
    ]
    
    if use_joint_limits
        joint_params = [
        (Revolute, (:origin, 1)),
        (PlanarAxis, (1, 2)),
        (PlanarAxis, (2, 3)),
        (PlanarAxis, (3, 1))
    ]
    else
        joint_params = [
            (Revolute, (:origin, 1)),
            (Revolute, (1, 2)),
            (Revolute, (2, 3)),
            (Revolute, (3, 1))
        ]
    end
    
    joint_vertices = [
        ([0, 0, 0], [0, 0, -link_length/2]),
        ([0, 0, link_length/2], [0, 0, -link_length/2]),
        ([0, 0, link_length/2], [0, 0, -link_length/2]),
        ([0, 0, link_length/2], [0, 0, -link_length/2])
    ]
    
    return create_mechanism(link_params, joint_params, joint_vertices, use_joint_limits=use_joint_limits, joint_limit=joint_limit, density=MATERIAL_DENSITY)
end

"""
Control function for the mechanism simulation.
"""
function control!(mechanism, t)

    # get impulses for all joints
    for i in 1:length(mechanism.joints)
        println("max impulse Joint$i: ", maximum(mechanism.joints[i].impulses[2]))
    end

    freeids = Set(Dojo.getid.(mechanism.bodies))
    z = get_maximal_configuration(mechanism, freeids)
    # initial_obj = objective_function(mechanism, freebodies, z, dz.*t)
    res = get_residual(mechanism, freeids, z)

    println("Joint Residual: ", maximum(abs.(res)))
    # Apply a small force to the first link to try to induce motion
    set_input!(mechanism.joints[2], [0.0, 0.0, -1.0])
end

mechanism = create_triangular_linkage()
z0 = initialize_mechanism!(mechanism)
mechanism = create_triangular_linkage(true, 1e-13)
set_maximal_state!(mechanism, z0)
vis = visualize_mechanism(mechanism, vis=vis, show_frame=false, show_joint=true, joint_radius=0.01)
storage, ke, pe, me = simulate_mechanism(mechanism, 100*mechanism.timestep, control_func=control!);
visualize_mechanism(mechanism, vis=vis, show_frame=false, show_joint=false, joint_radius=0.01, storage=storage)




"""
Check if the mechanism is jammed
"""
function is_jammed(mechanism, storage, threshold=1e-6)
    velocities = [norm(storage.v[i][end]) for i in 1:length(mechanism.bodies)]
    return all(v < threshold for v in velocities)
end

"""
Plot body positions and indicate jamming
"""
function plot_body_positions(storage, jammed, slop)
    num_bodies = length(storage.x)
    num_steps = length(storage.x[1])
    
    plot = plot(
        xlabel="X", ylabel="Y",
        title="Triangular Linkage (Slop: $slop, $(jammed ? "Jammed" : "Not Jammed"))",
        aspect_ratio=:equal,
        size=(800, 600)
    )
    
    for i in 1:num_bodies
        positions = [[storage.x[i][j][k] for j in 1:num_steps] for k in 1:2]
        plot!(positions..., 
            label="Link $i", 
            linewidth=2
        )
    end
    
    return plot
end

"""
Plot energy over time and indicate jamming
"""
function plot_energy(mechanism, ke, pe, me, jammed, slop)
    total_time = 0:mechanism.timestep:length(me)*mechanism.timestep-mechanism.timestep
    
    plot = plot(title="Energy (Slop: $slop, $(jammed ? "Jammed" : "Not Jammed"))", 
                xlabel="Time (s)", ylabel="Energy (J)")
    plot!(total_time, me, label="Mechanical Energy")
    plot!(total_time, pe, label="Potential Energy")
    plot!(total_time, ke, label="Kinetic Energy")
    
    return plot
end

# Main execution
for slop in SLOP_VALUES
    mechanism = create_triangular_linkage(slop)
    initialize_mechanism!(mechanism)
    
    simulation_time = 5.0  # Simulate for 5 seconds
    storage, ke, pe, me = simulate_mechanism(mechanism, simulation_time)
    
    jammed = is_jammed(mechanism, storage)
    
    # Plot results
    position_plot = plot_body_positions(storage, jammed, slop)
    display(position_plot)
    savefig(position_plot, "triangular_positions_slop_$(slop).png")
    
    energy_plot = plot_energy(mechanism, ke, pe, me, jammed, slop)
    display(energy_plot)
    savefig(energy_plot, "triangular_energy_slop_$(slop).png")
    
    println("Slop: $slop - Jammed: $jammed")
end

# Create animation
anim = @animate for t in 1:length(storage.x[1])
    plot_layout = plot(layout=(1,1), size=(800, 600), aspect_ratio=:equal)
    for i in 1:length(storage.x)
        plot!(plot_layout, [storage.x[i][t][1]], [storage.x[i][t][2]], 
            label="", 
            markersize=6, 
            markerstrokewidth=0,
            seriestype=:scatter
        )
        if i < length(storage.x)
            next_i = i + 1
        else
            next_i = 1
        end
        plot!(plot_layout, [storage.x[i][t][1], storage.x[next_i][t][1]], 
                           [storage.x[i][t][2], storage.x[next_i][t][2]],
            label="",
            linewidth=2
        )
    end
    plot_layout
end every 5  # Update every 5 frames to reduce file size

gif(anim, "animated_triangular_linkage.gif", fps = 30)
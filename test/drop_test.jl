using Dojo
using DojoEnvironments
using LinearAlgebra

# Parameters
edge_length = 1.0 # 1 cm
mass = 1.0 # arbitrary mass
friction_coefficient = 0.9
external_force = 2.0 # 2 N in x-direction

# Create the mechanism
origin = Origin{Float64}()
box = Box(edge_length, edge_length, 2 * edge_length, mass; color = RGBA(0.8, 0.0, 0.0, 1.0), name = :block)

# Create joint constraint for the box (free to move in x and y directions, and rotate around z-axis)
joint = JointConstraint(Floating(origin, box), name=:floating)

mechanism = Mechanism(origin, [box], [joint]; gravity = [0, 0, -9.81], timestep = 0.001, input_scaling = 0.001)

# springs and dampers
springs = 0
dampers = 0
set_springs!(mechanism.joints, springs)
set_dampers!(mechanism.joints, dampers)

# Define contact points (8 corners of the box)
contact_origins = [
    [edge_length/2; edge_length/2; -edge_length],
    [edge_length/2; -edge_length/2; -edge_length],
    [-edge_length/2; edge_length/2; -edge_length],
    [-edge_length/2; -edge_length/2; -edge_length],
    [edge_length/2; edge_length/2; edge_length],
    [edge_length/2; -edge_length/2; edge_length],
    [-edge_length/2; edge_length/2; edge_length],
    [-edge_length/2; -edge_length/2; edge_length]
]

# Define normal vectors for each contact point
normals = fill(Z_AXIS,8)

# Define friction coefficients for each contact point
friction_coefficients = fill(friction_coefficient, 8)

contacts = contact_constraint(mechanism.bodies[1], normals;
        friction_coefficients = friction_coefficients,
        contact_origins = contact_origins, contact_radii = fill(0.025, 8),
    )



# Create the mechanism
mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, contacts; gravity=mechanism.gravity, timestep = mechanism.timestep, input_scaling = mechanism.input_scaling)

z0 = get_maximal_state(mechanism)
z0[3] = 2.0
set_maximal_state!(mechanism, z0)

# Controller function to apply the external force
function controller!(mechanism, k)
    for contact in mechanism.contacts
        # contact model
        model = contact.model

        # parent 
        pbody = get_body(mechanism, contact.parent_id)
        xp, vp, qp, ωp = Dojo.next_configuration_velocity(pbody.state, mechanism.timestep)

        # child
        cbody = get_body(mechanism, contact.child_id)
        xc, vc, qc, ωc = Dojo.next_configuration_velocity(cbody.state, mechanism.timestep)

        # distance 
        d = distance(model.collision, xp, qp, xc, qc)
        println("Contact Distance:", d)
        if d == 0.0
            println("Contact!")
        end
    end
    # Apply the external force in the x-direction near the top of the box
    return 0.0
    # add_external_force!(mechanism.bodies[1], force=[external_force, 0.0, 0.0], vertex=[0.0, 0.0, edge_length/2])
end

# Simulate
storage = simulate!(mechanism, 1.0, controller!, record = true, verbose = true)
# vis = Visualizer()
# Visualize
delete!(vis)
visualize(mechanism, storage, vis=vis, visualize_floor=true, show_contact=true, joint_radius=0.0)
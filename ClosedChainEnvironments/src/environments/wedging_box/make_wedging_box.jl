using Dojo
using LinearAlgebra

# Parameters
edge_length = 0.01 # 1 cm
mass = 0.1 # arbitrary mass
friction_coefficient = 0.9
external_force = 2.0 # 2 N in x-direction

# Create the mechanism
origin = Origin{Float64}()

# Create the wedge box
box = Box(edge_length, edge_length, 2 * edge_length, mass; color = RGBA(0.8, 0.0, 0.0, 1.0), name = :block)

# Create the bottom and top planes (represented as thin boxes)
plane_thickness = 0.001 # 1 mm thickness for planes
plane_width = 0.2 # 20 cm width for planes (adjust as needed)
bottom_plane = Box(plane_width, plane_width, plane_thickness, 1.0; color = RGBA(0.5, 0.5, 0.5, 0.5), name = :bottom_plane)
top_plane = Box(plane_width, plane_width, plane_thickness, 1.0; color = RGBA(0.5, 0.5, 0.5, 0.5), name = :top_plane)

# Create joint constraints
box_joint = JointConstraint(Floating(origin, box), name=:box_floating)
bottom_plane_joint = JointConstraint(Fixed(origin, bottom_plane, parent_vertex=[0, 0, -0.025]), name=:bottom_plane_fixed)
top_plane_joint = JointConstraint(Fixed(origin, top_plane, parent_vertex=[0, 0, 0.025]), name=:top_plane_fixed)

# Create the mechanism
mechanism = Mechanism(origin, [box, bottom_plane, top_plane], [box_joint, bottom_plane_joint, top_plane_joint];
                      gravity = [0, 0, -9.81], timestep = 0.001, input_scaling = 0.001)

# Set springs and dampers
springs = 0
dampers = 0
# set_springs!(mechanism.joints, springs)
# set_dampers!(mechanism.joints, dampers)

# Create contact models
sphere_radius = 0.0
box_bottom_contact_model = SphereBoxCollision{Float64,2,3,6}(
    [0.0, 0.0, -edge_length],  # origin_sphere (bottom of the box)
    plane_width, 
    plane_width, 
    plane_thickness,
    sphere_radius
)

box_top_contact_model = SphereBoxCollision{Float64,2,3,6}(
    [0.0, 0.0, edge_length],  # origin_sphere (top of the box)
    [-plane_width/2, -plane_width/2, -plane_thickness/2],  # origin_box_a (corner of top plane)
    [plane_width/2, plane_width/2, -plane_thickness/2],  # origin_box_b (opposite corner of top plane)
    sphere_radius
)

# Create contact constraints
box_bottom_contact = ContactConstraint(
    (box_bottom_contact_model, box.id, bottom_plane.id),
    name = :box_bottom_contact
)

box_top_contact = ContactConstraint(
    (box_top_contact_model, box.id, top_plane.id),
    name = :box_top_contact
)

# Update the mechanism with contacts
mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, [box_bottom_contact, box_top_contact];
                      gravity = mechanism.gravity, timestep = mechanism.timestep, input_scaling = mechanism.input_scaling)

# Set initial state
z0 = get_maximal_state(mechanism)
z0[3] = edge_length + 0.002 # Start 2 mm above the bottom plane
set_maximal_state!(mechanism, z0)

# Set positions for planes
set_maximal_state!(mechanism, bottom_plane, x=[0.0, 0.0, 0.0])
set_maximal_state!(mechanism, top_plane, x=[0.0, 0.0, 0.102]) # 102 mm above the bottom plane

# Controller function to apply the external force
function controller!(mechanism, k)
    for contact in mechanism.contacts
        model = contact.model
        pbody = get_body(mechanism, contact.parent_id)
        xp, vp, qp, ωp = Dojo.next_configuration_velocity(pbody.state, mechanism.timestep)
        cbody = get_body(mechanism, contact.child_id)
        xc, vc, qc, ωc = Dojo.next_configuration_velocity(cbody.state, mechanism.timestep)
        d = distance(model, xp, qp, xc, qc)
        if d ≈ 0.0
            println("Contact at $(contact.name)!")
        end
    end
    
    if k == 1
        println("Applying external force")
        add_external_force!(mechanism.bodies[1], force=[100*external_force, 0.0, 0.0], vertex=[0.0, 0.0, edge_length])
    end
end

# Simulate
opts = SolverOptions(verbose=false, rtol=1e-8, btol=1e-8, reg=1e-6, max_iter=100)
storage = simulate!(mechanism, 100*mechanism.timestep, controller!, record = true, opts = opts)

# Visualize
vis = Visualizer()
delete!(vis)
visualize(mechanism, storage, vis=vis, show_contact=true, joint_radius=0.0)
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
bottom_plane = Box(plane_width, plane_width, plane_thickness, 1.0; color = RGBA(0.5, 0.8, 0.5, 1.0), name = :bottom_plane)
top_plane = Box(plane_width, plane_width, plane_thickness, 1.0; color = RGBA(0.5, 0.5, 0.8, 1.0), name = :top_plane)

# Create joint constraints
box_joint = JointConstraint(Revolute(origin, box, [0, 1, 0]), name=:box_floating)
bottom_plane_joint = JointConstraint(Fixed(origin, bottom_plane, parent_vertex=[0, 0, -0.0105-plane_thickness/2]), name=:bottom_plane_fixed)
top_plane_joint = JointConstraint(Fixed(origin, top_plane, parent_vertex=[0, 0, 0.0105+plane_thickness/2]), name=:top_plane_fixed)

# Create the mechanism
mechanism = Mechanism(origin, [box, bottom_plane, top_plane], [box_joint, bottom_plane_joint, top_plane_joint];
                      gravity = [0, 0, -9.81], timestep = 0.001, input_scaling = 0.001)

# Set springs and dampers
springs = 0
dampers = 0
# set_springs!(mechanism.joints, springs)
# set_dampers!(mechanism.joints, dampers)

# Create contact models
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


sphere_radius = 0.0001
contacts = ContactConstraint{Float64}[]
for (i, contact_origin) in enumerate(contact_origins)
    box_contact_model1 = NonlinearContact{Float64,8}(friction_coefficient, Matrix{Float64}(I, 2,2), SphereBoxCollision{Float64,2,3,6}(
        contact_origin,  # origin_sphere (bottom of the box)
        plane_width, 
        plane_width, 
        plane_thickness,
        sphere_radius    
        ))
    
    box_contact_model2 = NonlinearContact{Float64,8}(friction_coefficient, Matrix{Float64}(I, 2,2), SphereBoxCollision{Float64,2,3,6}(
        contact_origin,  # origin_sphere (bottom of the box)
        plane_width, 
        plane_width, 
        plane_thickness,
        sphere_radius    
        ))


    # Create contact constraints
    box_bottom_contact = ContactConstraint(
        (box_contact_model1, box.id, bottom_plane.id),
        name = Symbol("box_bottom_contact$i")
    )

    box_top_contact = ContactConstraint(
        (box_contact_model2, box.id, top_plane.id),
        name = Symbol("box_top_contact$i")
    )

    contacts = [contacts; box_bottom_contact; box_top_contact]
end


# Update the mechanism with contacts
mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, contacts;
                      gravity = mechanism.gravity, timestep = mechanism.timestep, input_scaling = mechanism.input_scaling)

# Set initial state
z0 = get_maximal_state(mechanism)
# z0[3] = edge_length + 0.002 # Start 2 mm above the bottom plane
# set_maximal_state!(mechanism, z0)

# Set positions for planes
set_maximal_configurations!(get_body(mechanism, :bottom_plane), x=[0.0, 0.0, -0.0105-plane_thickness/2])
set_maximal_configurations!(get_body(mechanism, :top_plane), x=[0.0, 0.0, 0.0105+plane_thickness/2]) # 102 mm above the bottom plane
Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2
system = mechanism.system
@save "system0.jld2" system
# set_data!(mechanism, data)
# set_solution!(mechanism, sol)
Dojo.set_entries!(mechanism, reg=1e-10)
res = Dojo.full_vector(system)
A = Dojo.full_matrix(system)
vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false, show_contact=false, joint_radius=0.0)

# Controller function to apply the external force
# max_singular_values = []
using JLD2
function controller!(mechanism, k)
    for contact in mechanism.contacts
        model = contact.model
        pbody = get_body(mechanism, contact.parent_id)
        xp, vp, qp, ωp = Dojo.next_configuration_velocity(pbody.state, mechanism.timestep)
        cbody = get_body(mechanism, contact.child_id)
        xc, vc, qc, ωc = Dojo.next_configuration_velocity(cbody.state, mechanism.timestep)
        d = distance(model.collision, xp, qp, xc, qc)
        if d ≈ 0.0
            println("Contact at $(contact.name)!")
        end
    end

    # if k % 2 == 0
    save("system$(1500+k).jld2", "system", mechanism.system)
    # end

    # if k == 1
        # println("Applying external force")
    add_external_force!(mechanism.bodies[1], force=[external_force, 0.0, 0.0], vertex=[0.0, 0.0, edge_length])
    # end
end

# Simulate
opts = SolverOptions(verbose=false, rtol=1e-8, btol=1e-8, reg=1e-6, max_iter=100)
storage = simulate!(mechanism, 500*mechanism.timestep, controller!, record = true, opts = opts)

# Visualize
vis = Visualizer()
delete!(vis)
visualize(mechanism, storage, vis=vis,visualize_floor=false, show_contact=true, joint_radius=0.0)

save("wedge_block_1501.jld2", "mechanism", mechanism, "storage", storage)

for contact in mechanism.contacts
    model = contact.model
    pbody = get_body(mechanism, contact.parent_id)
    xp, vp, qp, ωp = Dojo.next_configuration_velocity(pbody.state, mechanism.timestep)
    cbody = get_body(mechanism, contact.child_id)
    xc, vc, qc, ωc = Dojo.next_configuration_velocity(cbody.state, mechanism.timestep)
    d = distance(model.collision, xp, qp, xc, qc)
    # println(d)
    if d < 1e-4
        println("Contact at $(contact.name)!")
        println(maximum(contact.impulses[1]))
    end
end


Dojo.pull_residual!(mechanism)               # store the residual inside mechanism.residual_entries
Dojo.ldu_factorization!(mechanism.system)    # factorize system, modifies the matrix in place
A = full_matrix(mechanism.system)
F = svd(A, full=true, alg=LinearAlgebra.QRIteration())
rank = sum(F.S .> 1e-6)
println("rank: ", rank)
println("min eigen", maximum(F.S))
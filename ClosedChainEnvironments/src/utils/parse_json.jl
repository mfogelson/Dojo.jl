using JSON
using Dojo

gravity = [0.0, 0.0, -9.81]
# Function to convert inertia values to matrix
function inertia_to_matrix(xx=1.0, yy=1.0, zz=1.0, xy=0.0, yz=0.0, xz=0.0)
    return [
        xx xy xz;
        xy yy yz;
        xz yz zz
    ]
end

# Function to create a Mesh body
function create_mesh_body(link_name, link_data)
    inertia = inertia_to_matrix(link_data["inertia"]...)
    rot_mat = [link_data["xaxis"] link_data["yaxis"] link_data["zaxis"]]
    q = Dojo.QuatRotation(rot_mat)
    body = Mesh(
        link_data["stl"],
        # 0.001,
        link_data["mass"], # kg
        inertia .* 1e-4, # kg cm^2 to kg m^2
        # Dojo.I(3) * 0.001,
        position_offset= -Vector{Float64}(link_data["stl_orig"])/100.0, # cm to m zeros(Float64, 3),
        orientation_offset=Quaternion(1, 0, 0, 0),
        scale=ones(3) * 0.001, # mm to m 
        name=Symbol(link_name)
    )
    
    set_maximal_configurations!(body, x=link_data["x"]/100.0, q=q.q)
    return body
end

# Function to create a joint constraint
function create_joint_constraint(joint_name, joint_data, bodies, slop=0.0, damper=0.0, rot_joint_limits=[szeros(Float64,0), szeros(Float64,0)])
    parent_body = findfirst(x -> x.name == Symbol(joint_data["parent"]), bodies)
    child_body = findfirst(x -> x.name == Symbol(joint_data["child"]), bodies)

    if joint_data["type"] == "Revolute"
        lower_limit = szeros(Float64, 0)
        upper_limit = szeros(Float64, 0)
        if joint_data["lower_limit"] != []
            lower_limit = SVector{1}(joint_data["lower_limit"])

        end
        if joint_data["upper_limit"] != []
            upper_limit = SVector{1}(joint_data["upper_limit"])
        end
        # lower_limit = ifelse(joint_data["lower_limit"] != [], SVector{1}(joint_data["lower_limit"]), szeros(Float64, 0))
        # upper_limit = ifelse(joint_data["upper_limit"] != [], SVector{1}(joint_data["upper_limit"]), szeros(Float64, 0))
        if !iszero(slop)
            joint = JointConstraint(
                PlanarAxis(
                    bodies[parent_body], bodies[child_body],
                    Dojo.vector_rotate(joint_data["axis"], bodies[child_body].state.q2'),;
                    parent_vertex=joint_data["parent_vertex"]/100.0,
                    child_vertex=joint_data["child_vertex"]/100.0,
                    orientation_offset=bodies[parent_body].state.q2' * bodies[child_body].state.q2,
                    rot_joint_limits=rot_joint_limits,
                    damper=damper,
                    # rot_joint_limits=[lower_limit, upper_limit], 
                    tra_joint_limits=[[-slop, -slop], [slop, slop]]
                ), name=Symbol(joint_name)
            )
        else
            println("Creating Revolute joint")
            joint = JointConstraint(
                Revolute(
                    bodies[parent_body], bodies[child_body],
                    Dojo.vector_rotate(joint_data["axis"], bodies[child_body].state.q2'),;
                    parent_vertex=joint_data["parent_vertex"]/100.0,
                    child_vertex=joint_data["child_vertex"]/100.0,
                    orientation_offset=bodies[parent_body].state.q2' * bodies[child_body].state.q2,
                    rot_joint_limits=rot_joint_limits,
                    damper=damper,
                    # rot_joint_limits=[lower_limit, upper_limit]
                ), name=Symbol(joint_name)
            )
        end
    elseif joint_data["type"] == "Fixed"
        joint = JointConstraint(
            Fixed(
                bodies[parent_body], bodies[child_body],
                parent_vertex=joint_data["parent_vertex"]/100.0,
                child_vertex=joint_data["child_vertex"]/100.0,
                orientation_offset=bodies[parent_body].state.q2' * bodies[child_body].state.q2,
            ), name=Symbol(joint_name)
        )
    end

    return joint
end

# Function to create contact constraints
function create_contact_constraints(bodies, contact_feet)
    contacts = ContactConstraint{Float64}[]
    friction_coefficients = [0.8]

    if contact_feet
        contact_bodies = [bodies[1]] #[get_body(mechanism, name) for name in body_names]
        n = length(contact_bodies)
        normals = fill(Z_AXIS, n)
        contact_radii = [0.1]
        contact_origins = [[1.27; -3.53; -0.0]]
        # body = contact_bodies[1] #get_body(mechanism, body_names[1])
        
        # model = Dojo.NonlinearContact(body, normals[1], friction_coefficient; contact_origin=[1.0; -3.0; -0.0], contact_radius=contact_radii[1])
        # contact = ContactConstraint((model, body.id, 0); name=:floor_contact)
        contacts = [contacts; contact_constraint(contact_bodies, normals; friction_coefficients, contact_origins, contact_radii, names=[:floor_contact])]
    end

    return contacts
end

function reflect_rigid_body!(body, plane_normal, plane_center)
    # Normalize the plane normal vector
    plane_normal = plane_normal / Dojo.norm(plane_normal)

    # Translate the position vector relative to the plane's center
    relative_position = body.state.x2 - plane_center
    
    # Reflect the position vector about the plane
    reflected_relative_position = relative_position - 2 * Dojo.dot(relative_position, plane_normal) * plane_normal
    
    # Translate the reflected position back to the original coordinate system
    reflected_position = reflected_relative_position + plane_center
    
    # Reflect the attitude quaternion about the plane
    reflected_attitude_quaternion = body.state.q2

    set_maximal_configurations!(body, x=reflected_position, q=reflected_attitude_quaternion)
    
    return nothing
end

function update_all_body_states!(bodies, translation_offset, rotation_offset)
    for body in bodies
        x = Dojo.vector_rotate(body.state.x2, rotation_offset) + translation_offset
        q = rotation_offset * body.state.q2
        set_maximal_configurations!(body, x=x, q=q)
    end
end

function parse_json(filename; translation_offset=zeros(3), rotation_offset=Dojo.RotX(0.0), contact_feet=false, slop=0.0, gravity=[0.0, 0.0, -9.81],  damper=0.001, timestep=0.001, rot_joint_limits_even=[szeros(Float64,0), szeros(Float64,0)],rot_joint_limits_odd=[szeros(Float64,0), szeros(Float64,0)])

    parsed_data = JSON.parsefile(filename)

    # Initialize components
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

   
    # Extract link data and create bodies
    for (link_name, link_data) in parsed_data["links"]
        if link_data["stl"] != ""
            body = create_mesh_body(link_name, link_data)
            push!(bodies, body)
        end
    end

    # Apply offsets to bodies
    update_all_body_states!(bodies, translation_offset, rotation_offset)

    # Extract joint data and create joints
    for (i, (joint_name, joint_data)) in enumerate(parsed_data["joints"])
        joint = create_joint_constraint(joint_name, joint_data, bodies, slop, damper, iseven(i) ? rot_joint_limits_even : rot_joint_limits_odd)
        push!(joints, joint)
    end

    # mechanism = Mechanism(origin, bodies, joints)

    contacts = create_contact_constraints(bodies, contact_feet)

    return Mechanism(origin, bodies, joints, contacts, timestep=timestep, gravity=gravity), contacts
end

function copy_bodies(bodies, translation_offset=zeros(3), rotation_offset=Dojo.RotX(0.0), plane_center=zeros(3), plane_normal=nothing, name_ext="_new")
    new_bodies = Body{Float64}[]
    for body in bodies
        new_body = Body(
            body.mass,
            body.inertia,
            shape=body.shape,
            name=Symbol(String(body.name)*name_ext)
        )
        set_maximal_configurations!(new_body, x=copy(body.state.x2), q=copy(body.state.q2))

        if !isnothing(plane_normal)
            reflect_rigid_body!(new_body, plane_normal, plane_center)
        end

        push!(new_bodies, new_body)
    end

    update_all_body_states!(new_bodies, translation_offset, rotation_offset)
    return new_bodies
end

function run_local()
    # using MeshIO     
    # using GeometryBasics

    # # Load the STL file
    # mesh = mechanism.bodies[1].shape
    # ext = lowercase(splitext(mesh.path)[2])
    # mesh = Dojo.MeshFileGeometry(open(read, mesh.path), ext[2:end])
    # # mesh = Dojo.load(.path)

    # # Extract vertices from the mesh
    # vertices = GeometryBasics.coordinates(mesh)
    # using MeshIO, GeometryBasics

    # function parse_binary_stl(mesh)
    #     contents = mesh.contents

    #     # Skip the first 80 bytes of header
    #     header = contents[1:80]

    #     # Get the number of triangles (next 4 bytes)
    #     num_triangles = reinterpret(UInt32, contents[81:84])[1]

    #     println("Number of triangles: ", num_triangles)

    #     # Start reading the triangles
    #     triangles = []
    #     offset = 85

    #     for i in 1:num_triangles
    #         # Read normal vector (12 bytes)
    #         normal = reinterpret(Float32, contents[offset:offset+11])
    #         offset += 12

    #         # Read 3 vertices (36 bytes total: 12 bytes per vertex)
    #         v1 = reinterpret(Float32, contents[offset:offset+11])
    #         v2 = reinterpret(Float32, contents[offset+12:offset+23])
    #         v3 = reinterpret(Float32, contents[offset+24:offset+35])
    #         offset += 36

    #         # Read the attribute byte count (2 bytes, ignored)
    #         attribute_byte_count = reinterpret(UInt16, contents[offset:offset+1])[1]
    #         offset += 2

    #         # Store the triangle (you can store it as a tuple or create a GeometryBasics object)
    #         push!(triangles, (v1, v2, v3))
    #     end

    #     return triangles
    # end

    # Load the STL file using MeshIO
    # mesh = load("path_to_your_file.stl")

    # Parse the binary STL
    # triangles = parse_binary_stl(mesh)

    # # println("Parsed triangles: ", triangles[1:5])  # Display first 5 triangles
    # # function compute_bounding_box_size(triangles)
    # #     # Flatten all the vertices into one array
    # #     vertices = vcat([vcat([tri[1], tri[2], tri[3]]) for tri in triangles]...)

    # #     # Find the minimum and maximum points for each axis
    # #     min_point = [minimum([vertex[i] for vertex in vertices]) for i in 1:3]
    # #     max_point = [maximum([vertex[i] for vertex in vertices]) for i in 1:3]

    # #     # Compute bounding box size (difference between max and min)
    # #     bounding_box_size = max_point .- min_point

    # #     return bounding_box_size, min_point, max_point
    # # end

    # bounding_box_size, min_point, max_point = compute_bounding_box_size(triangles)
    # println("Bounding Box Size: ", bounding_box_size)
    # Load the JSON file
    #! Scissor
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_09_12_scissor_mechanism/scissor_mech_20links v3_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/10_ZeroG_Flight/Dojo_description/Short Linkage Assembly v6_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/10_ZeroG_Flight/Dojo_description/Scissor_2Link/Short Linkage Assembly v8_link_dict.json"
    filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/10_ZeroG_Flight/Dojo_description/Scissor_4Link/four_link_short_scissor v11_link_dict.json"
    #! Bennett Linkage
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_09_09_bennett_linkage/Bennett-Linkage v6_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_09_22_pt2_bennett_linkage/Bennett-Basis v18_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_09_23_bennett_linkage/Bennett-Basis v18_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/10_ZeroG_Flight/Dojo_description/Bennett Linkage/Bennett-Basis v18_link_dict.json"
    #! PET
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_09_23_pet/PET_unit v80_link_dict.json"
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_08_01_PET_unit/PET/PET_unit v60_link_dict.json"

    #! Jansen
    # filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_07_24_jansen/Jansen Mechanism v7 v7_link_dict.json" #/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/2024_07_24_pet/folding_scissor_assembly2 v5_link_dict.json" #"/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/Experiments/Jansen Mechanism v7 v6_link_dict.json" #"/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/test_body v5_link_dict.json" #"/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/09_Closed_Loop_Simulation/fusion_to_dojo_test_description/robot_config.json" #"/Users/mitchfogelson/Jansen_description/robot_config.json" #"/Users/mitchfogelson/Projects/Research_Projects/fusion2urdf/Test/orientaiton_description/robot_config.json"

    # translation_offset = [0.0, 0.0, 4.9]
    # rotation_offset = Dojo.RotX(pi/2)
    mechanism, contact = parse_json(filename, slop=0.0)#, translation_offset, rotation_offset, true);

    mechanism.origin
    # fixed_body = get_body(mechanism, Symbol("Short Linkage v8:1"))
    # fixed_body = get_body(mechanism, Symbol("Component5:1"))
    # fixed_body = get_body(mechanism, Symbol("Bennett-Basis:1"))
    # fixed_body = get_body(mechanism, Symbol("Bennett_base:1"))
    # fixed_body = get_body(mechanism, Symbol("Long_scissor_unit v5:1+Long_link_member v3:1")) #Symbol("Component8:1"))
    # joint = JointConstraint(Fixed(mechanism.origin, fixed_body, parent_vertex=fixed_body.state.x2, orientation_offset=fixed_body.state.q2), name=:fixed)
    # joint = JointConstraint(Revolute(mechanism.origin, fixed_body, [0,0,1], parent_vertex=fixed_body.state.x2, orientation_offset=fixed_body.state.q2), name=:fixed)
    # joints = [joint; mechanism.joints[1:end]]
    # mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints, mechanism.contacts, gravity=gravity)
    vis = Visualizer()
    delete!(vis)
    visualize(mechanism, vis=vis, visualize_floor=false, show_frame=false, show_joint=true, show_contact=true, joint_radius=0.01)
    #! Check the size of the bodies and the inertias
    #! add initial velocity to the bodies

    #! Simulate the bodies 
    #! Track key positions
    # get joints Revolute 8 to 23
    # Function to retrieve joints from Revolute 8 to Revolute 23 based on name pattern
    # function get_revolute_joints(mechanism)
    #     # List to store the matching joints
    #     matching_joints = JointConstraint{Float64}[]
    #     # Iterate through the desired revolute numbers
    #     for i in 8:65
    #         # Convert the revolute number to a string pattern to search for
    #         search_pattern = "Revolute $i"
    #         # Iterate through all joints in the mechanism
    #         for joint in mechanism.joints # Assuming joints are accessible as keys in the mechanism
    #             # Check if the joint name contains the desired pattern
    #             if occursin(search_pattern, string(joint.name))
    #                 # Retrieve and store the joint
    #                 push!(matching_joints, joint)
    #             end
    #         end
    #     end
    #     return matching_joints
    # end
    # # get_joint(mechanism, Symbol("Revolute 8_ComponentX:1"))
    # reduced_joints = get_revolute_joints(mechanism)
    # function get_bodies_from_joints(mechanism, joints)
    #     bodies = Body{Float64}[]
    #     for joint in joints 
    #         cbody = get_body(mechanism, joint.child_id)
    #         pbody = get_body(mechanism, joint.parent_id)
    #         if !in(cbody, bodies)
    #             push!(bodies, cbody)
    #         end
    #         if !in(pbody, bodies)
    #             push!(bodies, pbody)
    #         end
    #     end
    #     return bodies
    # end
    # reduced_bodies = get_bodies_from_joints(mechanism, reduced_joints)
    # mechanism = Mechanism(mechanism.origin, reduced_bodies, [joint; reduced_joints], mechanism.contacts, gravity=gravity)
    # # new_bodies = copy_bodies(mechanism.bodies, zeros(3), Dojo.RotX(0.0), get_body(mechanism, Symbol("Component18:1")).state.x2, X_AXIS, "_new")
    # # push!(joints, joint)

    # # mechanism = Mechanism(mechanism.origin, [mechanism.bodies; new_bodies], mechanism.joints, mechanism.contacts, gravity=[0.0, 0.0, -9.81])
    # initial_angle = pi/3
    # num_sets = 2
    # link_length = 4.5/2
    # for i in 1:num_sets
    #     Dojo.set_maximal_configurations!(mechanism.bodies[2i-1], x=mechanism.bodies[2i-1].state.x2 + [(i-1)*link_length*cos(initial_angle/2), 0, 0], q=Dojo.RotY((-1)^(i+i%2)*initial_angle/2))
    #     Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=mechanism.bodies[2i].state.x2 + [(i-1)*link_length*cos(initial_angle/2), 0, 0], q=Dojo.RotY((-1)^(i+((i+1)%2))*initial_angle/2))
    # end
    # get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])

    # vis = Visualizer()
    # delete!(vis)
    # visualize(mechanism, vis=vis, visualize_floor=false, show_frame=false, show_joint=true, show_contact=true, joint_radius=0.01)

    res = Dojo.residual(mechanism)
    for joint in mechanism.joints
        println(joint.name)
        println(Dojo.norm(Dojo.constraint(mechanism, joint)))
    end
    mechanism.timestep = 0.001
    # Run the simulation
    # Define a PD controller function for velocity control
    function velocity_controller!(joint, desired_velocity, actual_velocity, kp, kd)
        # Calculate velocity error
        error = desired_velocity - actual_velocity

        # PD control: compute control input (torque/acceleration) based on error
        control_input = kp * error - kd * actual_velocity

        # Set the computed input (torque/acceleration) to the joint
        input = zeros(Dojo.input_dimension(joint))
        println("Control Input: $control_input")
        # println("Error: $error")
        input[end] = control_input
        set_input!(joint, input)
    end

    # Main controller function for the mechanism
    function controller!(mechanism::Mechanism, t)
        # Print joint residual to monitor constraints
        # println("Joint Residual: $(Dojo.norm(Dojo.residual(mechanism)))")

        # Retrieve the specific joint for control, e.g., the 4th joint
        joint = mechanism.joints[3]
        # joint = mechanism.joints[4]  # Adjust index as needed for the correct joint

        # Get the current joint angle and velocity
        joint_angle = Dojo.minimal_coordinates(mechanism, joint)[end]
        pbody = get_body(mechanism, joint.parent_id)
        cbody = get_body(mechanism, joint.child_id)
        joint_velocity = Dojo.minimal_velocities(joint, pbody, cbody, mechanism.timestep)[end]  # Assuming this function retrieves joint velocity

        # Print the current joint angle and velocity for monitoring
        # println("Joint Angle: $joint_angle")
        # println("Joint Velocity: $joint_velocity")
        # for joint in mechanism.joints
        #     println(Dojo.norm(joint.impulses[2]))
        # end
        # Desired velocity (set this as per your control objective)
        desired_velocity = -0.523599 # Rad/sec  # Desired velocity value

        # PD controller gains (tune these values based on your mechanism's response)
        kp = 10.0  # Proportional gain
        kd = 0.1   # Derivative gain

        # Apply velocity control using the PD controller
        velocity_controller!(joint, desired_velocity, joint_velocity, kp, kd)
    end

    function pos_controller!(mechanism, k)
        # Target angle in radians
        target_angle = pi/4

        # Get the current angle between the two bodies
        _, current_angle, _ = get_angle_between_bodies(mechanism.bodies[1], mechanism.bodies[2])

        println("Current angle: ", current_angle)

        # Calculate the error
        angle_error = target_angle - current_angle
        link_length = 0.45

        vel_error = -mechanism.bodies[1].state.ω15[2]
        println("Angle error: ", angle_error)

        # Proportional gain (you may need to tune this)
        Kp = 1e3
        Kd = 7e2

        # Calculate the control torque
        control_torque = Kp * -angle_error + Kd * vel_error *0.1
        control_force = control_torque/link_length/2

        println("Control torque: ", control_torque)
        # println("Control force: ", control_force)

        # Apply the control torque to both bodies in opposite directions around x-axis
        add_external_force!(mechanism.bodies[1], force=[0.0, 0.0, 0.0], torque=[0.0, -control_torque/2, 0.0])
        add_external_force!(mechanism.bodies[2], force=[0.0, 0.0, 0.0], torque=[0.0, control_torque/2, 0.0])

        # You can keep the original forces if needed, or modify them
        # add_external_force!(mechanism.bodies[1], force=[0, control_force/2, 0], vertex=[0, 0, -link_length/2])
        # add_external_force!(mechanism.bodies[2], force=[0, -control_force/2, 0], vertex=[0, 0, -link_length/2])
    end
    # for (i, body) in enumerate(mechanism.bodies)
    #     set_maximal_configurations!(body, x=storage.x[i][1], q=storage.q[i][1])
    # end
    # Dojo.zero_velocities!(mechanism)
    # mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, mechanism.contacts, gravity=zeros(3))
    # z = storage[1].
    zero_velocities!(mechanism)
    # opts = SolverOptions(rtol=1e-4, btol=1e-5, verbose=true, max_iter=15, undercut=10.0)
    opts = SolverOptions(rtol=1e-6, btol=1e-5, verbose=true, max_iter=20)
    steps = 1:300
    storage = Storage(steps, length(mechanism.bodies))
    # @time simulate!(mechanism, 1:2, storage, record=true, opts=opts)
    simulate!(mechanism, steps, storage, pos_controller!, record=true, opts=opts)

    # using JLD2
    # save("09_15_2024_scissor_20_link_jamming.jld2", "storage", storage, "mechanism", mechanism, "opts", opts, "steps", steps)
    # using Profile
    # using ProfileView
    # @profile simulate!(mechanism, 1:2, storage, controller!, record=true, opts=opts)
    # ProfileView.view()
    # vis = Visualizer()
    visualize(mechanism, storage, vis=vis, show_frame=true, visualize_floor=false, show_joint=true, show_contact=true, joint_radius=0.01)

    A = full_matrix(mechanism.system)
    b = Dojo.full_vector(mechanism.system)
    F = Dojo.svd(A, full=true, alg=Dojo.LinearAlgebra.QRIteration())
    rank = sum(F.S .> 1e-6)
    F.S
    # more tick marks
    # using Plots
    plot(F.S, yscale=:log10, legend=false, xlabel="Singular Value Index", ylabel="Singular Value", title="Singular Value Spectrum", xticks=0:1:length(F.S), yticks=10.0 .^ (-10:1:10))
    # plot_body_positions_comparison([storage], [0.001])
    # ============================================================================ # maximal_to_json
    # function matrix_to_inertia(mat) where T
    #     return (mat[1, 1], mat[2, 2], mat[3, 3], -mat[1, 2], -mat[1, 3], -mat[2, 3])
    # end

    # # Function to convert Mechanism to JSON
    # function mechanism_to_json(mechanism::Mechanism)
    #     bodies = []
    #     joints = []

    #     # Convert bodies to JSON-compatible format
    #     for body in mechanism.bodies
    #         inertia_vec = matrix_to_inertia(body.inertia)
    #         body_json = Dict(
    #             "name" => String(body.name),
    #             "stl" => body.shape.path,
    #             "mass" => body.mass,
    #             "inertia" => inertia_vec,
    #             "x" => collect(body.state.x2),
    #             "q" => collect([body.state.q2.s, body.state.q2.v1, body.state.q2.v2, body.state.q2.v3])
    #         )
    #         push!(bodies, body_json)
    #     end

    #     # Convert joints to JSON-compatible format
    #     for joint in mechanism.joints
    #         joint_json = Dict(
    #             "name" => String(joint.name),
    #             "type" => String(joint.type),
    #             "parent" => String(get_body(mechanism, joint.parent_id).name),
    #             "child" => String(get_body(mechanism, joint.child_id).name),
    #             "axis" => collect(joint.rotational.axis),
    #             "parent_vertex" => collect(joint.translational.vertices[1]),
    #             "child_vertex" => collect(joint.translational.vertices[2]),
    #             "orientation_offset" => collect(joint.rotational.orientation_offset),
    #             "upper_limit" => joint.rotational.joint_limits[1],
    #             "lower_limit" => joint.rotational.joint_limits[2]
    #         )
    #         push!(joints, joint_json)
    #     end

    #     # Create final JSON-compatible dictionary
    #     mechanism_json = Dict(
    #         "links" => Dict([body["name"] => body for body in bodies]),
    #         "joints" => Dict([joint["name"] => joint for joint in joints]),
    #     )

    #     return JSON.json(mechanism_json, 4)
    # end

    # # save json file 
    # json_data = mechanism_to_json(mechanism)
    # open("mechanism.json", "w") do io
    #     println(io, json_data)
    # end
end

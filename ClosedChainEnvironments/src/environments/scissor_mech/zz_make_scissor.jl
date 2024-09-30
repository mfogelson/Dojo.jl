using Dojo

function make_body(unit_thickness::Float64, unit_length::Float64, unit_mass::Float64, name::Symbol, color)
    return Cylinder(unit_thickness, unit_length, unit_mass, name=name, color=color)
end

function make_joint(parent::Body, child::Body, rotation_axis::Vector{Float64}, parent_vertex::Vector{Float64}, child_vertex::Vector{Float64}, name::Symbol)
    return JointConstraint(Revolute(parent, child, rotation_axis; parent_vertex=parent_vertex, child_vertex=child_vertex, orientation_offset=Dojo.RotY(pi)), name=name)
end

function initialize_mechanism()
    unit_thickness = 0.1
    unit_length = 1.0
    unit_mass = 1.0

    link1 = make_body(unit_thickness, unit_length, unit_mass, :link1, RGBA(1.0, 0, 0))
    link2 = make_body(unit_thickness, unit_length, unit_mass, :link2, RGBA(0, 1.0, 0))
    link3 = make_body(unit_thickness, unit_length, unit_mass, :link1, RGBA(1.0, 0, 0))
    link4 = make_body(unit_thickness, unit_length, unit_mass, :link2, RGBA(0, 1.0, 0))
    bodies = [link1, link2, link3, link4]
    joint1 = make_joint(link1, link2, [0, 1.0, 0.], zeros(3), zeros(3), :joint1)
    joint2 = make_joint(link3, link4, [0, 1.0, 0.], zeros(3), zeros(3), :joint2)
    joint3 = make_joint(link2, link3, [0, 1.0, 0.], [0.0, 0.0, unit_length/2], [0.0, 0.0, -unit_length/2], :joint3)
    joint4 = make_joint(link1, link4, [0, 1.0, 0.], [0.0, 0.0, unit_length/2], [0.0, 0.0, -unit_length/2], :joint4)
    joints = [joint1, joint2, joint3, joint4]

    
    origin = Origin()
    mechanism = Mechanism(origin, bodies, joints)
end

function initialize_mechanism(n::Int)

    unit_thickness = 0.1
    unit_length = 1.0
    unit_mass = 1.0

    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]

    # Create the links
    for i in 1:(2*n)
        color = i % 2 == 1 ? RGBA(1.0, 0, 0) : RGBA(0, 1.0, 0)
        push!(bodies, make_body(unit_thickness, unit_length, unit_mass, Symbol("link$i"), color))
    end

    # Create the joints between pairs of links
    for i in 1:n
        link_a = bodies[2*i-1]
        link_b = bodies[2*i]
        push!(joints, make_joint(link_a, link_b, [0, 1, 0.], zeros(3), zeros(3), Symbol("joint_pairs$i")))
    end

    # Create the joints between alternating pairs of links
    for i in 1:(n-1)
        link_a = bodies[2*i-1]
        link_b = bodies[2*i+1]
        push!(joints, make_joint(link_a, link_b, [0, 1, 0.], [0.0, 0.0, unit_length/2], [0.0, 0.0, -unit_length/2], Symbol("joint_left$i")))
    end

    # Create the cross joints
    for i in 1:(n-1)
        link_a = bodies[2*i]
        link_b = bodies[2*i+2]
        push!(joints, make_joint(link_a, link_b, [0, 1, 0.], [0.0, 0.0, unit_length/2], [0.0, 0.0, -unit_length/2], Symbol("joint_right$i")))
    end

    print(length(joints))

    origin = Origin()
    return Mechanism(origin, bodies, joints)
end

mechanism = initialize_mechanism(20)


vis = Visualizer()
visualize(mechanism, vis=vis, visualize_floor=false)
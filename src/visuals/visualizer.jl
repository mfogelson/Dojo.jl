"""
    visualize(mechanism, storage; vis, build, show_contact, animation, color, name)

    visualize mechanism using trajectory from storage 

    mechanism: Mechanism 
    storage: Storage 
    vis: Visualizer 
    build: flag to construct mechanism visuals (only needs to be built once)
    show_contact: flag to show contact locations on system 
    color: RGBA 
    name: unique identifier for mechanism
"""
function visualize(mechanism::Mechanism, storage::Storage{T,N}; vis::Visualizer=Visualizer(),
    build::Bool=true, 
    show_joint=false,
    joint_radius=0.1,
    show_contact=false, 
    animation=nothing, 
    color=nothing, 
    name::Symbol=:robot) where {T,N}

    storage = deepcopy(storage)
    bodies = mechanism.bodies
    origin = mechanism.origin

    # Build robot in the visualizer
    build && build_robot(mechanism, 
        vis=vis, 
        show_joint=show_joint,
        show_contact=show_contact, 
        color=color, 
        name=name)

    # Create animations
    framerate = Int64(round(1/mechanism.timestep))
    (animation == nothing) && (animation =
        MeshCat.Animation(Dict{MeshCat.SceneTrees.Path,MeshCat.AnimationClip}(), framerate))

    # Bodies and Contacts
    for (id, body) in enumerate(bodies)
        shape = body.shape
        visshape = convert_shape(shape)
        subvisshape = nothing
        showshape = false
        if visshape !== nothing
            subvisshape = vis[name][:bodies][Symbol(body.name, "__id_$id")]
            showshape = true
        end

        animate_node!(storage, id, shape, animation, subvisshape, showshape)

        if show_joint
            for (jd, joint) in enumerate(mechanism.joints)
                if joint.child_id == body.id
                    radius = 0.1
                    joint_shape = Sphere(radius, 
                        position_offset=joint.translational.vertices[2],
                        color=RGBA(0.0, 0.0, 1.0, 0.5))
                    visshape = convert_shape(joint_shape)
                    subvisshape = nothing
                    showshape = false
                    if visshape !== nothing
                        subvisshape = vis[name][:joints][Symbol(joint.name, "__id_$(jd)")]
                        showshape = true
                    end
                    animate_node!(storage, id, joint_shape, animation, subvisshape, showshape)
                end
            end
        end

        if show_contact
            for (jd, contact) in enumerate(mechanism.contacts)
                if contact.parent_id == body.id
                    radius = abs(contact.model.collision.contact_radius)
                    (radius == 0.0) && (radius = 0.01)
                    contact_shape = Sphere(radius,
                        position_offset=contact.model.collision.contact_origin, #TODO: generalize for collision checking
                        orientation_offset=one(Quaternion), 
                        color=RGBA(1.0, 0.0, 0.0, 0.5))
                    visshape = convert_shape(contact_shape)
                    subvisshape = nothing
                    showshape = false
                    if visshape !== nothing
                        subvisshape = vis[name][:contacts][Symbol(contact.name, "__id_$(jd)")]
                        showshape = true
                    end
                    animate_node!(storage, id, contact_shape, animation, subvisshape, showshape)
                end
            end
        end
    end

    # Origin
    id = origin.id
    shape = origin.shape
    visshape = convert_shape(shape)
    if visshape !== nothing
        subvisshape = vis[name][:bodies][Symbol(:origin, "_id")]
        shapetransform = transform(szeros(T,3), one(Quaternion{T}), shape)
        settransform!(subvisshape, shapetransform)
    end

    setanimation!(vis, animation)
    return vis, animation
end

"""
    build_robot(mechanism; vis, show_contact, name, color)

    construct visuals for mechanism 

    mechanism: Mechanism 
    vis: Visualizer 
    show_contact: flag to show contact locations on mechanism 
    name: unique identifier 
    color: RGBA
"""
function build_robot(mechanism::Mechanism; 
    vis::Visualizer=Visualizer(),
    show_joint=false,
    joint_radius=0.1,
    show_contact=false, 
    name::Symbol=:robot, 
    color=nothing) where {T,N}

    bodies = mechanism.bodies
    origin = mechanism.origin
    # set_background!(vis)
    # set_light!(vis)
    # set_floor!(vis)

    # Bodies and Contacts
    for (id, body) in enumerate(bodies)
        if color !== nothing
            shape = deepcopy(body.shape)
            set_color!(shape, color)
        else
            shape = body.shape
        end
        visshape = convert_shape(shape)
        subvisshape = nothing
        if visshape !== nothing
            subvisshape = vis[name][:bodies][Symbol(body.name, "__id_$id")]
            setobject!(subvisshape, visshape, shape, 
                transparent=(show_joint || show_contact))
        end

        if show_joint
            for (jd, joint) in enumerate(mechanism.joints)
                if joint.child_id == body.id
                    radius = joint_radius
                    joint_shape = Sphere(radius,
                        position_offset=joint.translational.vertices[2],
                        color=RGBA(0.0, 0.0, 1.0, 0.5))
                    visshape = convert_shape(joint_shape)
                    subvisshape = nothing
                    if visshape !== nothing
                        subvisshape = vis[name][:joints][Symbol(joint.name, "__id_$(jd)")]
                        setobject!(subvisshape, visshape, joint_shape, 
                            transparent=false)
                    end
                end
            end
        end

        if show_contact
            for (jd, contact) in enumerate(mechanism.contacts)
                if contact.parent_id == body.id
                    radius = abs(contact.model.collision.contact_radius)
                    (radius == 0.0) && (radius = 0.01)
                    contact_shape = Sphere(radius,
                        position_offset=(contact.model.collision.contact_origin),
                        orientation_offset=one(Quaternion), color=RGBA(1.0, 0.0, 0.0, 0.5))
                    visshape = convert_shape(contact_shape)
                    subvisshape = nothing
                    if visshape !== nothing
                        subvisshape = vis[name][:contacts][Symbol(contact.name, "__id_$(jd)")]
                        setobject!(subvisshape,visshape,contact_shape,transparent=false)
                    end
                end
            end
        end
    end

    # Origin
    id = origin.id
    shape = origin.shape
    visshape = convert_shape(shape)
    if visshape !== nothing
        subvisshape = vis[name][:bodies][Symbol(:origin, "_id")]
        setobject!(subvisshape,visshape,shape,transparent=(show_joint || show_contact))
    end
    return vis
end

"""
add_axis!
"""
function add_axes!(vis,axes_name, scale, R; head_l = 0.1, head_w = 0.05, r = zeros(3), q = [1.0, 0.0, 0.0, 0.0])
	red_col =RGBA(1.0,0.0,0.0,1.0)
	green_col =RGBA(0.0,1.0,0.0,1.0)
	blue_col =RGBA(0.0,0.0,1.0,1.0)

	cylx = GeometryBasics.Cylinder(Point(0,0,0.0), Point(scale,0,0.0), R)
	cyly = GeometryBasics.Cylinder(Point(0,0,0.0), Point(0,scale,0.0), R)
	cylz = GeometryBasics.Cylinder(Point(0,0,0.0), Point(0,0.0,scale), R)

    setobject!(vis[axes_name][:x], cylx, MeshPhongMaterial(color=red_col))
	head = Cone(Point(scale,0,0.0), Point(scale + head_l, 0.0, 0), head_w)
	setobject!(vis[axes_name][:hx], head, MeshPhongMaterial(color=red_col))

	setobject!(vis[axes_name][:y], cyly, MeshPhongMaterial(color=green_col))
	head = Cone(Point(0,scale,0.0), Point(0, scale + head_l, 0.0), head_w)
	setobject!(vis[axes_name][:hy], head, MeshPhongMaterial(color=green_col))

	setobject!(vis[axes_name][:z], cylz, MeshPhongMaterial(color=blue_col))
	head = Cone(Point(0,0.0,scale), Point(0, 0.0, scale + head_l), head_w)
	setobject!(vis[axes_name][:hz], head, MeshPhongMaterial(color=blue_col))

    settransform!(vis[axes_name],Translation(r) ∘ LinearMap(QuatRotation(q)))

	return nothing
end


"""
    set_robot(vis, mechanism, z; show_contact, name)

    visualze mechanism configuration from maximal representation 

    vis: Visualizer 
    mechanism: Mechanism 
    z: maximal state 
    show_contact: flag to show contact locations on mechanism 
    name: unique identifier
"""
function set_robot(vis::Visualizer, mechanism::Mechanism, z::Vector{T};
    show_joint::Bool=false,
    joint_radius=0.1,
    show_contact::Bool=true, 
    show_tf::Bool=true,
    name::Symbol=:robot) where {T,N}

    (length(z) == minimal_dimension(mechanism)) && (z = minimal_to_maximal(mechanism, z))
    bodies = mechanism.bodies
    origin = mechanism.origin

    # Bodies and Contacts
    for (id, body) in enumerate(bodies)
        x, _, q, _ = unpack_maximal_state(z, id)
        shape = body.shape
        visshape = convert_shape(shape)
        subvisshape = nothing
        showshape = false
        if visshape !== nothing
            subvisshape = vis[name][:bodies][Symbol(body.name, "__id_$id")]
            showshape = true
        end

        set_node!(x, q, id, shape, subvisshape, showshape)

        if show_joint
            for (jd, joint) in enumerate(mechanism.joints)
                if joint.child_id == body.id
                    # radius = joint_radius
                    # joint_shape = Sphere(radius,
                    #     position_offset=joint.translational.vertices[2],
                    #     color=RGBA(0.0, 0.0, 1.0, 0.5))
                    # # joint_shape = Triad(x, q)
                    # visshape = convert_shape(joint_shape)
                    # subvisshape = nothing
                    # showshape = false
                    # if visshape !== nothing
                    #     subvisshape = vis[name][:joints][Symbol(joint.name, "__id_$(jd)")]
                    #     showshape = true
                    # end
                    # set_node!(x, q, id, joint_shape, subvisshape, showshape)
                    add_axes!(vis, joint.name, 1, 1, r=x, q=q)
                end
            end
        end


        if show_contact
            for (jd, contact) in enumerate(mechanism.contacts)
                if contact.parent_id == body.id
                    radius = abs(contact.model.collision.contact_radius)
                    (radius == 0.0) && (radius = 0.01)
                    contact_shape = Sphere(radius,
                        position_offset=(contact.model.collision.contact_origin),
                        orientation_offset=one(Quaternion), color=RGBA(1.0, 0.0, 0.0, 0.5))
                    visshape = convert_shape(contact_shape)
                    subvisshape = nothing
                    showshape = false
                    if visshape !== nothing
                        subvisshape = vis[name][:contacts][Symbol(contact.name, "__id_$(jd)")]
                        showshape = true
                    end
                    set_node!(x, q, id, contact_shape, subvisshape, showshape)
                end
            end
        end
    end

    # Origin
    id = origin.id
    shape = origin.shape
    visshape = convert_shape(shape)
    if visshape !== nothing
        subvisshape = vis[name][:bodies][Symbol(:origin, "_id")]
        shapetransform = transform(szeros(T,3), one(Quaternion{T}), shape)
        settransform!(subvisshape, shapetransform)
    end
    return vis
end

function transform(x, q, shape)
    scale_transform = MeshCat.LinearMap(diagm(shape.scale))
    x_transform = MeshCat.Translation(x + vector_rotate(shape.position_offset, q))
    q_transform = MeshCat.LinearMap(q * shape.orientation_offset)
    return MeshCat.compose(x_transform, q_transform, scale_transform)
end

MeshCat.js_scaling(s::AbstractVector) = s
MeshCat.js_position(p::AbstractVector) = p

function set_node!(x, q, id, shape, shapevisualizer, showshape) where {T,N}
    if showshape
        # TODO currently setting props directly because MeshCat/Rotations doesn't convert scaled rotation properly.
        # If this changes, do similarily to origin
        setprop!(shapevisualizer, "scale", MeshCat.js_scaling(shape.scale))
        setprop!(shapevisualizer, "position", MeshCat.js_position(x + vector_rotate(shape.position_offset, q)))
        setprop!(shapevisualizer, "quaternion", MeshCat.js_quaternion(q * shape.orientation_offset))
    end
    return
end

function animate_node!(storage::Storage{T,N}, id, shape, animation, shapevisualizer, showshape) where {T,N}
    for i=1:N
        x = storage.x[id][i]
        q = storage.q[id][i]
        atframe(animation, i) do
            set_node!(x, q, id, shape, shapevisualizer, showshape)
        end
    end
    return
end

function MeshCat.setobject!(subvisshape, visshape, shapes::Shapes; 
    transparent=false)
    for (i, s) in enumerate(shapes.shape)
        v = subvisshape["node_$i"]
        setobject!(v, visshape[i], s, transparent=transparent)
        scale_transform = MeshCat.LinearMap(diagm(s.scale))
        x_transform = MeshCat.Translation(s.position_offset)
        q_transform = MeshCat.LinearMap(s.orientation_offset)
        t = MeshCat.compose(x_transform, q_transform, scale_transform)
        settransform!(v, t)
    end
end

function MeshCat.setobject!(subvisshape, visshape, shape::Shape; 
    transparent=false)
    setobject!(subvisshape, visshape, MeshPhongMaterial(color=(transparent ? RGBA(0.75, 0.75, 0.75, 0.5) : shape.color)))
end

function MeshCat.setobject!(subvisshape, visshape::Vector, shape::Capsule; transparent=false)
    setobject!(subvisshape["cylinder"], visshape[1], MeshPhongMaterial(color=(transparent ? RGBA(0.75, 0.75, 0.75, 0.5) : shape.color)))
    setobject!(subvisshape["cap1"], visshape[2], MeshPhongMaterial(color=(transparent ? RGBA(0.75, 0.75, 0.75, 0.5) : shape.color)))
    setobject!(subvisshape["cap2"], visshape[3], MeshPhongMaterial(color=(transparent ? RGBA(0.75, 0.75, 0.75, 0.5) : shape.color)))
end

function MeshCat.setobject!(subvisshape, visshape, shape::Mesh; transparent=false)
    if visshape.mtl_library == ""
        visshape = MeshFileGeometry(visshape.contents, visshape.format)
        setobject!(subvisshape, visshape, MeshPhongMaterial(color=shape.color))
    else
        setobject!(subvisshape, visshape)
    end
end

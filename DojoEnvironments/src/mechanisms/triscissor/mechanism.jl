function get_triscissor(;
    timestep=0.01,
    input_scaling=timestep, 
    gravity=-9.81,
    mass=1,
    radius=0.05,
    len=1.0, 
    num_sets=2,
    scale=0.2,
    color=RGBA(0.9, 0.9, 0.9, 1.0),
    springs=0,
    dampers=0, 
    joint_limits=Dict(),
    keep_fixed_joints=true, 
    friction_coefficient=0.4,
    contact=true,
    contact_type=:nonlinear,
    T=Float64)

    # mechanism
    origin = Origin{T}(name=:origin)

    # Initialize bodies and joints array 
    links = Body{T}[]
    joints = JointConstraint{T}[]
    body_position = []
    body_orientation = []

    p1ᴮ = [-len; 0; 0] 

    p2ᴮ = [len; 0; 0]

    len = 2*len
    for i in 1:num_sets
        link = Body{T}[]
        joint = JointConstraint{T}[]
        # create set of 2 links
        for j in 1:3
            for ll in 1:2
                if j ==1 
                    θ0 = Quaternion(Dojo.Lmat(Dojo.RotY(pi/2))*Dojo.Lmat(Dojo.RotX(-pi/2))*Dojo.vector(Dojo.RotZ((-1)^ll*pi/4))) #UnitQuaternion(RotYXZ(pi/2, -pi/2, (-1)^ll*pi/4)).q
                elseif j == 3
                    θ0 = Quaternion(Dojo.Lmat(Dojo.RotY(pi/2))*Dojo.Lmat(Dojo.RotX(pi/6))*Dojo.vector(Dojo.RotZ((-1)^ll*pi/4))) #UnitQuaternion(RotYXZ(pi/2, pi/6, (-1)^ll*pi/4)).q
                else
                    θ0 = Quaternion(Dojo.Lmat(Dojo.RotY(pi/2))*Dojo.Lmat(Dojo.RotX(-pi/6+pi))*Dojo.vector(Dojo.RotZ((-1)^ll*pi/4))) #UnitQuaternion(RotYXZ(pi/2, -pi/6+pi, (-1)^ll*pi/4)).q
                end
                
                # p1ᴮ[2] = -p1ᴮ[2]
                # end
                r = [len/2*(cos(pi/6+2pi/3*(j-1))+cos(pi/6+2pi/3*(j))), len/2*(sin(pi/6+2pi/3*(j-1))+sin(pi/6+2pi/3*(j))), (i-1)*len/2*sqrt(2)]
                # if i%2 == 0
                append!(link, [Cylinder(radius, len, mass, name=Symbol("cylinder_", i, j, ll))])
                append!(body_position, [r])

                append!(body_orientation, [θ0])
                # println(set_maximal_configurations!(link[end], x=r, q=θ0))
                # update_state!(link[end], 0)
                # println(link[end].state)
                        # Link(r=r, q=[θ0.s, θ0.v1, θ0.v2, θ0.v3], length=length, a=-p2ᴮ, b=-p1ᴮ)]) 

                # else
                #     append!(link, [Link(r=r, q=[θ0.s, θ0.v1, θ0.v2, θ0.v3], length=length, a=p1ᴮ, b=p2ᴮ)]) #for j in 1:3])
                # end
            end
            
            # Kt_ = SMatrix{3, 3}(diagm([1.0, 1.0, 0.0]))
            # Ct_ = SMatrix{3, 3}(diagm([1.0, 1.0, 0.0]))
            append!(joint, [JointConstraint(Revolute(link[end-1], link[end], [0, 0, 1], parent_vertex=zeros(3), child_vertex=zeros(3)))])
        

        end
        # print(link[1].bᴮ)
        append!(links, link)

        # Top joints (1, 4) (2, 5) (3, 6)
        append!(joint, [JointConstraint(Spherical(link[ind[1]], link[ind[2]], parent_vertex=[0, 0, -len/2], child_vertex=[0, 0, -len/2])) for ind in [(1, 4), (2, 5), (3, 6)]])


        # # Bottom joints (1, 6) (2, 3) (4, 5)
        append!(joint, [JointConstraint(Spherical(link[ind[1]], link[ind[2]], parent_vertex=[0, 0, len/2], child_vertex=[0, 0, len/2])) for ind in [(1, 6), (2, 3), (4, 5)]])

        # connect those sets with the previous set
        if i > 1
            append!(joint, [JointConstraint(Spherical(links[ind[1]+6*(i-2)], links[ind[2]+6*(i-2)], parent_vertex=[0, 0, -len/2], child_vertex=[0, 0, len/2])) for ind in [(1, 8), (3, 10), (5, 12)]])
            append!(joint, [JointConstraint(Spherical(links[ind[1]+6*(i-2)], links[ind[2]+6*(i-2)], parent_vertex=[0, 0, -len/2], child_vertex=[0, 0, len/2])) for ind in [(1, 9), (3, 11), (5,7)]])

            append!(joint, [JointConstraint(Spherical(links[ind[1]+6*(i-2)], links[ind[2]+6*(i-2)], parent_vertex=[0, 0, -len/2], child_vertex=[0, 0, len/2])) for ind in [(2, 7), (4, 9), (6,11) ]])
            append!(joint, [JointConstraint(Spherical(links[ind[1]+6*(i-2)], links[ind[2]+6*(i-2)], parent_vertex=[0, 0, -len/2], child_vertex=[0, 0, len/2])) for ind in [(2, 12), (4, 8), (6,10) ]])
        end

        append!(joints, joint)
    end
    
    # bodies = [
    #     Sphere(radius, mass; name=:sphere1, color),
    #     Sphere(radius*scale, mass*scale^3; name=:sphere2, color)
    # ]
    # bodies[1].inertia = Diagonal([1.9, 2.1, 2])

    # joints = [
    #     JointConstraint(Floating(origin, bodies[1]); name=:floating_joint),
    #     JointConstraint(Fixed(bodies[1], bodies[2];
    #         parent_vertex=[0,0,radius]), name = :fixed_joint)
    # ]
    print(length(links))
    mechanism = Mechanism(origin, links, joints;
        timestep, gravity, input_scaling)

    # springs and dampers
    set_springs!(mechanism.joints, springs)
    set_dampers!(mechanism.joints, dampers)

    # joint limits    
    joints = set_limits(mechanism, joint_limits)
    mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints;
        gravity, timestep, input_scaling)
    
    # contacts
    contacts = ContactConstraint{T}[]

    if contact
        n = length(links)
        normals = fill(Z_AXIS,n)
        friction_coefficients = fill(friction_coefficient,n)
        contact_radii = [radius;radius*scale]
        contacts = [contacts;contact_constraint(links, normals; friction_coefficients, contact_radii, contact_type)]
    end

    mechanism = Mechanism(mechanism.origin, mechanism.bodies, mechanism.joints, contacts;
        gravity, timestep, input_scaling)

    print(length(mechanism.bodies))

    # zero configuration
    initialize_triscissor!(mechanism, body_position=body_position, body_orientation=body_orientation)

    # construction finished
    return mechanism
end

function initialize_triscissor!(mechanism::Mechanism{T};
    body_position=[zeros(3) for _ in 1:length(mechanism.bodies)], body_orientation=[one(Quaternion) for _ in 1:length(mechanism.bodies)]) where T

    # zero_velocities!(mechanism)


    # zero_coordinates!(mechanism)

    for (body, body_pose, body_orient) in zip(mechanism.bodies, body_position, body_orientation)
        println(body_pose)
        println(body_orient)
        set_maximal_configurations!(body, x=body_pose, q=body_orient)
    end

end

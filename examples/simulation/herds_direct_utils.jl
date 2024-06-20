using Dojo
using Quaternions
using Rotations
include("scissor_direct_utils.jl")

function get_quaternion_between_points(point1::Array{Float64}, point2::Array{Float64})
       # Create vectors from points
       v1 = Dojo.normalize(point1)
       v2 = Dojo.normalize(point2)
   
       # Compute the cross product and the angle between the vectors
       axis = Dojo.cross(v1, v2)
       cos_angle = Dojo.dot(v1, v2)
   
       # Handle the case when vectors are parallel or anti-parallel
       if cos_angle ≈ -1
           # Vectors are anti-parallel
           # Find a perpendicular vector to use as the rotation axis
           axis = perpendicular_vector(v1)
           q = QuatRotation(RotMatrix(AngleAxis(pi, axis...))).q
           return Quaternion(q.s, q.v1, q.v2, q.v3)
       elseif cos_angle ≈ 1
           # Vectors are parallel
           return Quaternion(1.0, 0.0, 0.0, 0.0) # No rotation needed
       else
           # General case
           sin_angle = sqrt(1.0 - cos_angle^2)
           angle = acos(cos_angle)
           q = QuatRotation(RotMatrix(AngleAxis(angle, axis...))).q
           return Quaternion(q.s, q.v1, q.v2, q.v3)
       end
   end
   
function perpendicular_vector(v::AbstractVector)
    # Find a non-zero component of v
    if abs(v[1]) > eps(Float64)
        return normalize(cross(v, [0, 1, 0]))
    else
        return normalize(cross(v, [1, 0, 0]))
    end
end

function get_herds(num_cells, kres_radius, beam_length, unit_width, unit_thickness, unit_length, unit_mass)
    origin = Origin()
    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    # subassembly = Mechanism{Float64}[]

    # Create plates
    for n in 1:num_cells+1
        plate = Cylinder(kres_radius, unit_thickness, unit_mass, name=Symbol("plate_$(n)"))
        set_maximal_configurations!(plate, x=[0, 0, (n-1)*4*beam_length])
        push!(bodies, plate)
    end

    # Create base cylinder
    # base = Cylinder(kres_radius, unit_thickness, unit_mass, name=:base, color=RGBA(0.75, 0.0, 0.0))
    # push!(bodies, base)
    # base_joint = JointConstraint(Spherical(origin, base), name=:Fixed_Base_Joint)
    # push!(joints, base_joint)

    # Create top cylinder
    # top = Cylinder(kres_radius, unit_thickness, unit_mass, name=:top, color=RGBA(0.75, 0.0, 0.0))
    # push!(bodies, top)
    for n in 1:num_cells
        base = bodies[n]
        
        top = bodies[n+1]
        for i in 1:2
            for type in ["short"]
                println(Dojo.getGlobalID())
                dir = type == "short" ? 1.0 : -1.0

                θ_bot = (i-1)*pi/3
                parent_vertex_bottom = [(kres_radius+dir*unit_thickness)*cos(θ_bot), (kres_radius+dir*unit_thickness)*sin(θ_bot), unit_thickness/2]

                θ_top = type == "short" ? (i)*-pi/3 : (i)*-pi/3-pi/3
                parent_vertex_top = [(kres_radius+dir*unit_thickness)*cos(θ_top), (kres_radius+dir*unit_thickness)*sin(θ_top), -unit_thickness/2]

                # Create scissor body and joint
                body, joint = get_scissor(beam_length, unit_width, unit_thickness, unit_length, unit_mass; name_ext="$(type)_member$(i)", color=RGBA(1.0, 0.75, 0.75, 0.5))

                body = deepcopy(body)
                joint = deepcopy(joint)

                temp_mechanism = deepcopy(Mechanism(Origin(), deepcopy(body),  deepcopy(joint)))
                # append!(subassembly, temp_mechanism)
                print("HERE")
                set_scissor_configuration!(temp_mechanism, zeros(3), pi/4)
                out = initialize_constraints!(temp_mechanism, 
                fixedids=[temp_mechanism.joints[1].parent_id, temp_mechanism.joints[1].child_id],
                regularization=0.0,
                lineIter=10, 
                newtonIter=20,
                debug=true,
                ε =1e-6)
                println(out)

                position_desired = parent_vertex_bottom + [0, 0, (n-1)*4*beam_length]

                orientation_desired = get_quaternion_between_points(parent_vertex_bottom + [0, 0, (n-1)*4*beam_length], parent_vertex_top + [0, 0, (n)*4*beam_length])

                bottom_end = temp_mechanism.bodies[end-3]
                top_end = temp_mechanism.bodies[end]
                move_scissor(temp_mechanism, (position_desired-bottom_end.state.x2), (bottom_end.state.q2'), zeros(3))


                push!(bodies, deepcopy(temp_mechanism.bodies)...)
                push!(joints, deepcopy(temp_mechanism.joints)...)
                for joint in joints
                    println(joint.name)
                    println(joint.parent_id)
                    println(joint.child_id)
                end

                # find body in bodies with the folling name
                bottom_end = bodies[findfirst(body -> body.name == Symbol("$(type)_member$(i)_left_end_fixed"), bodies)]
                println(bottom_end.name)
                
                pitch_joint = JointConstraint(
                    Spherical(base, bottom_end; parent_vertex=parent_vertex_bottom), 
                    name=Symbol("Spherical_Joint_Beam_$(type)_to_bottom$(i)")
                ) 
                push!(joints, pitch_joint)


                top_end = bodies[findfirst(body -> body.name == Symbol("$(type)_member$(i)_right_end_fixed"), bodies)]
                pitch_joint = JointConstraint(
                    Spherical(top, top_end; parent_vertex=parent_vertex_top), 
                    name=Symbol("Spherical_Joint_Beam_$(type)_to_top$(i)")
                ) 
                push!(joints, pitch_joint)
          
            end
        end
    end

    mechanism = Mechanism(origin, bodies, joints)

    return mechanism
end


# ### Scissor Parameter[s
unit_width = 0.266/4
unit_thickness = 0.00266*5
unit_length = 0.25
unit_mass = 0.01
beam_length = 0.266
diag_length = beam_length #sqrt(2*beam_length^2*(1 - cos(deg2rad(80))))
rotation_axis = [1; 0; 0]
# ### Kresling Parameters

kres_radius = beam_length

mechanism = get_herds(1, kres_radius, beam_length, unit_width, unit_thickness, unit_length, unit_mass);

if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.03)

initialize_constraints!(mechanism, 
    fixedids=[mechanism.bodies[1].id, mechanism.bodies[2].id],
    regularization=0.0,
    lineIter=10, 
    newtonIter=20,
    debug=true,
    ε =1e-6)

for joint in mechanism.joints
    println(joint.name)
    println(get_body(mechanism, joint.parent_id).name)
    println(get_body(mechanism, joint.child_id).name)
end

for body in mechanism.bodies
    println(body.name)
end

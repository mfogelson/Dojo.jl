using Dojo

"""
Create a scissor mechanism with the specified parameters.

Parameters:
- num_sets: Number of scissor units (default: 3)
- initial_angle: Initial angle of the mechanism in radians (default: 0.0)
- slop: Amount of play in the joints in meters (default: 0.0)
- link_length: Length of each link in meters (default: 0.045)
- link_mass: Mass of each link in kg (default: 0.0015)
- link_radius: Radius of each cylindrical link in meters (default: 0.003)
- damper: Damping coefficient for the joints (default: 0.001)

Returns:
- mechanism: Dojo Mechanism object representing the scissor linkage
"""
function get_scissor_mechanism(;
    num_sets=3,
    initial_angle=0.0,
    slop=0.0,
    link_length=0.045,
    link_mass=0.0015,
    link_radius=0.003,
    damper=0.001, 
    rotation_axis=[1; 0; 0], 
    gravity=[-9.81, 0, 0],
    timestep=0.001,
    rot_joint_limits_even=[[-π+0.02], [0.02]],
    rot_joint_limits_odd=[[0.02], [π-0.02]]
)
    origin = Origin()

    bodies = Body{Float64}[]
    joints = JointConstraint{Float64}[]
    loop_joints = JointConstraint{Float64}[]

    for i in 1:num_sets
        body = Cylinder(link_radius, link_length, link_mass, name=Symbol("link$(2*i-1)"))
        body2 = Cylinder(link_radius, link_length, link_mass, name=Symbol("link$(2*i)"))

        push!(bodies, body, body2)

        if i == 1
            joint1 = JointConstraint(Revolute(origin, body, rotation_axis), name=Symbol("origin_joint"))
            joint2 = JointConstraint(Revolute(body, body2, rotation_axis, rot_joint_limits=rot_joint_limits_odd), name=Symbol("joint_pairs$i"))
            push!(joints, joint1, joint2)
        else
            if iszero(slop)
                joint1 = JointConstraint(Revolute(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
                joint2 = JointConstraint(Revolute(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2]))
            else
                tra_joint_limits = [[-slop/2, -slop/2], [slop/2, slop/2]]
                joint1 = JointConstraint(PlanarAxis(bodies[end-3], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], tra_joint_limits=tra_joint_limits, damper=damper))
                joint2 = JointConstraint(PlanarAxis(bodies[end-2], body2, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], tra_joint_limits=tra_joint_limits, damper=damper))
            end
            push!(joints, joint1, joint2)

            if iszero(slop)
                joint3 = JointConstraint(Revolute(body, body2, rotation_axis, rot_joint_limits=iseven(i) ?  rot_joint_limits_even : rot_joint_limits_odd, damper=damper), name=Symbol("joint_pairs$i"))
            else
                tra_joint_limits = [[-slop/2, -slop/2], [slop/2, slop/2]]
                joint3 = JointConstraint(PlanarAxis(body, body2, rotation_axis; tra_joint_limits=tra_joint_limits, rot_joint_limits=iseven(i) ?  rot_joint_limits_even : rot_joint_limits_odd, damper=damper), name=Symbol("joint_pairs$i"))
            end

            push!(loop_joints, joint3)
        end
    end

    append!(joints, loop_joints)
    mechanism = Mechanism(origin, bodies, joints, timestep=timestep, gravity=gravity)

    for i in 1:num_sets
        Dojo.set_maximal_configurations!(mechanism.bodies[2i-1], x=[0, 0, (i-1)*link_length*cos(initial_angle/2)], q=Dojo.RotX((-1)^(i+1)*initial_angle/2))
        Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=[0, 0, (i-1)*link_length*cos(initial_angle/2)], q=Dojo.RotX((-1)^(i)*initial_angle/2))
    end

    return mechanism
end


# ### Setup
# PKG_SETUP
using Dojo
using Plots
# ### Parameters
radius = 0.1
link_length = 1
mass = 1
rotation_axis = [1;0;0] 
connection = [0;0;0]
damper = 10
num_sets = 10

# ### Mechanism components
origin = Origin()

# ### Need to specify the vector type in Julia
bodies = Body{Float64}[]
joints = JointConstraint{Float64}[]
loop_joints = JointConstraint{Float64}[]

# ### Construct Mechanism with num_sets links
for i in 1:num_sets
    # ### Create a cylinder body
    body = Cylinder(radius, link_length, mass, name=Symbol("link$(i)"))

    # ### Add the body to the bodies array
    push!(bodies, body)

    # ### Create a joint constraint
    if i == 1
        # ### Create a Revolute joint constraint between the origin and the first body
        joint1 = JointConstraint(Revolute(origin, body, rotation_axis; child_vertex=[0, 0, -link_length/2], damper=damper))
        push!(joints, joint1)
        
    elseif i == num_sets
        joint1 = JointConstraint(Revolute(bodies[end-1], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], damper=damper))

        push!(joints, joint1)

        # ### Create a Revolute joint constraint between the last body and the origin #! Note the offset of the parent_vertex
        joint3 = JointConstraint(Spherical(origin, body; parent_vertex=[0, link_length*num_sets-0.1, 0], child_vertex=[0, 0, link_length/2], damper=damper))
        push!(loop_joints, joint3)

    else
        # ### Create a Revolute joint constraint between the two bodies

        joint1 = JointConstraint(Revolute(bodies[end-1], body, rotation_axis; parent_vertex=[0, 0, link_length/2], child_vertex=[0, 0, -link_length/2], damper=damper))

        push!(joints, joint1)

    end
end

# ### Append the loop joints to the joints array
append!(joints, loop_joints)

# ### Construct Mechanism
mechanism = Mechanism(origin, bodies, joints, timestep=0.01)

# ### Set initial conditions
for i in 1:num_sets
    Dojo.set_maximal_configurations!(mechanism.bodies[i], x=[0, (i-1)*link_length+link_length/2, 0], q=Dojo.RotX(-pi/2))
    if i == num_sets
        Dojo.set_maximal_configurations!(mechanism.bodies[i], x=[0, (i-1)*link_length+link_length/2-0.1, 0], q=Dojo.RotX(-pi/2))
    end
    # Dojo.set_maximal_configurations!(mechanism.bodies[2i], x=[0, 0, (i-1)*link_length*cos(pi/4)], q=Dojo.RotX((-1)^(i)*pi/4))
end

# ### Set initial velocities
zero_velocities!(mechanism)
set_minimal_velocities!(mechanism, joints[end], [0, 0, 1.0])

# ### Set gravity
mechanism.gravity = [0, 0, -9.81] #zeros(3)

# ### Simulate
storage = simulate!(mechanism, 2.0, record=true)

# ### Visualizer #! Only run once
# vis = Visualizer()


# ### Visualize
delete!(vis)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_joint=true, show_frame=true)

# ### Plot Mechanical Energy
energy = Dojo.mechanical_energy(mechanism, storage)
pe = Dojo.potential_energy(mechanism, storage)

total_time = 0:mechanism.timestep:2.0-mechanism.timestep
plot(collect(total_time), energy, title="Mechanical Energy", xlabel="Time (s)", ylabel="Energy (J)", label="Mechanical Energy")
plot!(collect(total_time), pe, title="Mechanical Energy", xlabel="Time (s)", ylabel="Energy (J)", label="Potential Energy")


plot(collect(total_time)[1000:end], energy[1000:end], title="Mechanical Energy", xlabel="Time (s)", ylabel="Energy (J)")

# render(vis)
# ### Setup
# PKG_SETUP
using Dojo
using Pkg
Pkg.activate("DojoEnvironments")
using DojoEnvironments
Pkg.activate(".")
using JLD2

function create_mechanism(;radius, link_length, mass, plate_radius, plate_thickness, plate_mass, num_cells, timestep, gravity, max_extension, dampers)
    # ### Make system
    origin = Origin()
    plates = Body{Float64}[]
    short_members = [Body{Float64}[] for i in 1:num_cells]
    long_members = [Body{Float64}[] for i in 1:num_cells]
    joints = [JointConstraint{Float64}[] for i in 1:num_cells]

    # ### Make bodies
    # Plates
    for i in 1:num_cells+1
        plate = Cylinder(plate_radius, plate_thickness, plate_mass, name=Symbol("plate_$(i)"))

        push!(plates, plate)
    end

    # Members
    for i in 1:num_cells
        for j in 1:6
            # short member
            short1 = Cylinder(radius, link_length/2, mass, name=Symbol("bot_short$(j)_cell$(i)"))
            push!(short_members[i], short1)

            short2 = Cylinder(radius, link_length/2, mass, name=Symbol("top_short$(j)_cell$(i)"))
            push!(short_members[i], short2)

            # long member
            diag1 = Cylinder(radius, link_length/2, mass, name=Symbol("bot_diag$(j)_cell$(i)"))
            push!(long_members[i], diag1)

            diag2 = Cylinder(radius, link_length/2, mass, name=Symbol("top_diag$(j)_cell$(i)"))
            push!(long_members[i], diag2)
        end
    end

    # ### Make joints
    floating_joint = JointConstraint(Spherical(origin, plates[1]), name=Symbol("floating_joint"))
    push!(joints[1], floating_joint)

    for i in 1:num_cells
        base = plates[i]
        top = plates[i+1]
        bot_start = i % 2 == 1 ? 0 : 0 #-pi/3
        top_start = i % 2 == 1 ? -pi/3 : -pi/3 #0
        for j in 1:6
            # short member
            short_bottom = short_members[i][2*j-1]
            parent_vertex = [(base.shape.rh[1]+radius)*cos(-pi/3*(j)), (base.shape.rh[1]+radius)*sin(-pi/3*(j)), base.shape.rh[2]/2]
            child_vertex = [0, 0, -short_bottom.shape.rh[2]/2]

            joint = JointConstraint(Spherical(base, short_bottom; 
                                                    parent_vertex=parent_vertex, 
                                                    child_vertex=child_vertex), 
                                                    name=Symbol("joint_$(base.name)_to_$(short_bottom.name)"))
            push!(joints[i], joint)

            short_top = short_members[i][2*j]
            parent_vertex = [(top.shape.rh[1]+radius)*cos(-pi/3*(j-1)),(top.shape.rh[1]+radius)*sin(-pi/3*(j-1)), -top.shape.rh[2]/2]
            child_vertex = [0, 0, short_top.shape.rh[2]/2]

            joint = JointConstraint(Spherical(top, short_top; 
                                                    parent_vertex=parent_vertex, 
                                                    child_vertex=child_vertex), 
                                                    name=Symbol("joint_$(top.name)_to_$(short_top.name)"))
            push!(joints[i], joint)

            # short member prismatic
            parent_vertex = [0, 0, short_bottom.shape.rh[2]/2]
            child_vertex = [0, 0, -short_top.shape.rh[2]/2]

            diag_prismatic = JointConstraint(Prismatic(short_bottom, short_top, [0, 0, 1]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(short_bottom.name)_to_$(short_top.name)"))

            push!(joints[i], diag_prismatic)

            # long member bottom
            diag_bottom = long_members[i][2*j-1]
            bot_θ = bot_start + pi/3*(j-1)
            parent_vertex = [(base.shape.rh[1]-radius)*cos(-pi/3*(j-1)-2pi/3), (base.shape.rh[1]-radius)*sin(-pi/3*(j-1)-2pi/3), base.shape.rh[2]/2]
            child_vertex = [0, 0, -diag_bottom.shape.rh[2]/2]

            joint = JointConstraint(Spherical(base, diag_bottom; 
                                            parent_vertex=parent_vertex, 
                                            child_vertex=child_vertex), 
                                            name=Symbol("joint_$(base.name)_to_$(diag_bottom.name)"))
            push!(joints[i], joint)
            
            # long member top
            diag_top = long_members[i][2*j]
            top_θ = top_start + pi/3*(j-1)
            parent_vertex = [(top.shape.rh[1]-radius)*cos(-pi/3*(j-1)), (top.shape.rh[1]-radius)*sin(-pi/3*(j-1)), -top.shape.rh[2]/2]
            child_vertex = [0, 0, diag_top.shape.rh[2]/2]

            joint = JointConstraint(Spherical(top, diag_top; 
                                            parent_vertex=parent_vertex, 
                                            child_vertex=child_vertex), 
                                            name=Symbol("joint_$(top.name)_to_$(diag_top.name)"))
            push!(joints[i], joint)

            # long member prismatic
            parent_vertex = [0, 0, diag_bottom.shape.rh[2]/2]
            child_vertex = [0, 0, -diag_top.shape.rh[2]/2]

            diag_prismatic = JointConstraint(Prismatic(diag_bottom, diag_top, [0, 0, 1]; parent_vertex=parent_vertex, child_vertex=child_vertex), name=Symbol("joint_$(diag_bottom.name)_to_$(diag_top.name)"))

            push!(joints[i], diag_prismatic)
        end
    end

    all_bodies = vcat(plates, short_members..., long_members...)
    all_joints = vcat(joints...)

    mechanism = Mechanism(origin, all_bodies, all_joints, gravity=gravity, timestep=timestep)
    # ### initial structure guess 
    for i in 1:num_cells
        set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(i)")), x=[0, 0, (link_length+plate_thickness)*(i-1)])

        bot_start = i % 2 == 1 ? 0 : 0 #-pi/3
        top_start = i % 2 == 1 ? -pi/3 : -pi/3 #0

        for j in 1:6
            # short member
            set_maximal_configurations!(get_body(mechanism, Symbol("bot_short$(j)_cell$(i)")), x=[plate_radius*cos(-pi/3*(j)), plate_radius*sin(-pi/3*(j)), link_length/2+(link_length+plate_thickness)*(i-1)])
            set_maximal_configurations!(get_body(mechanism, Symbol("top_short$(j)_cell$(i)")), x=[plate_radius*cos(-pi/3*(j-1)), plate_radius*sin(-pi/3*(j-1)), link_length*3/2+(link_length+plate_thickness)*(i-1)])


            # long member
            bot_θ = bot_start + pi/3*(j-1)
            top_θ = top_start + pi/3*(j-1)

            set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag$(j)_cell$(i)")), x=[plate_radius*cos(-pi/3*(j-1)-2pi/3), plate_radius*sin(-pi/3*(j-1)-2pi/3), link_length/2+(link_length+plate_thickness)*(i-1)])

            set_maximal_configurations!(get_body(mechanism, Symbol("top_diag$(j)_cell$(i)")), x=[plate_radius*cos(-pi/3*(j-1)), plate_radius*sin(-pi/3*(j-1)), link_length*3/2+(link_length+plate_thickness)*(i-1)])
        end
    end
    set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(num_cells+1)")), x=[0, 0, (link_length/2+plate_thickness)*(num_cells)])

    plate_ids = [get_body(mechanism, Symbol("plate_$(i)")).id for i in [1]]
    res = initialize_constraints!(mechanism, 
                                        fixedids=plate_ids,
                                        regularization=0.0,
                                        lineIter=10, 
                                        newtonIter=10,
                                        debug=true,
                                        ε = 1e-6)

    # ### Set joint limits

    function get_joint_state(mechanism::Mechanism{T,Nn,Ne,Nb,Ni}, joint::JointConstraint) where {T,Nn,Ne,Nb,Ni} 
        c = zeros(T,0)
        v = zeros(T,0)
        pbody = get_body(mechanism, joint.parent_id)
        cbody = get_body(mechanism, joint.child_id)
        for element in (joint.translational, joint.rotational)
            pos = minimal_coordinates(element, pbody, cbody)
            vel = minimal_velocities(element, pbody, cbody, timestep)
            push!(c, pos...)
            push!(v, vel...)
        end
    
        return [c; v]
    end

    # joints = JointConstraint{Float64}[deepcopy(mechanism.joints)...]

    # for i in 1:num_cells
    #     for j in 1:6
    #         for (id, joint) in enumerate(joints)
    #             if (joint.name == Symbol("joint_bot_diag$(j)_cell$(i)_to_top_diag$(j)_cell$(i)") || joint.name == Symbol("joint_bot_short$(j)_cell$(i)_to_top_short$(j)_cell$(i)"))
    #                 # print("here")
    #                 println(joint.name, get_joint_state(mechanism, joint))

    #                 joints[id] = Dojo.add_limits(mechanism, joint,
    #                 tra_limits=[Dojo.SVector{1}(0.0), Dojo.SVector{1}(max_extension)])
    #             end
    #         end
    #     end
    # end

    # mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints; gravity, timestep)
    set_dampers!(mechanism.joints, dampers)

    return mechanism
end



# ### Parameters
# Link params
radius = 0.0127
link_length = 0.14605
max_extension = 1.1684
mass = 0.001 #TODO

# Plate params
plate_radius = 0.5842/2
plate_thickness = 0.00635
plate_mass = 0.001 #TODO

# Sim params
damper = 0.01
timestep = 0.01
reg = 1e-8
gravity = [0, 0, -9.81]

# System Params
num_cells = 2

mechanism_dict = Dict(:radius=>radius, :link_length=>link_length, :mass=>mass, :plate_radius=>plate_radius, :plate_thickness=>plate_thickness, :plate_mass=>plate_mass, :num_cells=>num_cells, :timestep=>timestep, :gravity=>gravity, :max_extension=>max_extension, :dampers=>damper)

mechanism = create_mechanism(;mechanism_dict...)

# set top plate height 
# set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(num_cells+1)")), x=[0, 0,0.75])

# initialize_constraints!(mechanism, fixedids=plate_ids, regularization=1e-5, lineIter=10, newtonIter=50, debud=true)
if isdefined(Main, :vis)
    # If it exists, delete it
    delete!(vis)
else
    # If it doesn't exist, initialize it as a Visualizer
    vis = Visualizer()
end
# vis = Visualizer()
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.01)

# get_joint_state(mechanism, mechanism.joints[4])
# get_minimal_state(mechanism)[]


# minimal_coordinates(mechanism.joints[4], mechanism.bodies[4], mechanism.bodies[5])
# # solve initialize_configuration! problem

# res = initialize_constraints!(mechanism, 
# fixedids=[],
# regularization=0.01,
# lineIter=10, 
# newtonIter=10,
# debug=true,
# ε = 1e-6)

function check_joint_name(joint, i, j)
    expected_name = "joint_bot_diag$(j)_cell$(i)_to_top_diag$(j)_cell$(i)"
    other_name = "joint_bot_short$(j)_cell$(i)_to_top_short$(j)_cell$(i)"
    return (string(joint.name) == expected_name || string(joint.name) == other_name)
end

function control!(mechanism, t)
    # get top body
    top = get_body(mechanism, Symbol("plate_$(num_cells+1)"))
    # desired height 
    desired_height = 1.0

    # get current height
    current_height = top.state.x2[3]

    # PID control
    kp = 10.0
    kd = 0.31
    ki = 0.0

    error = desired_height - current_height
    println(error)
    error_dot = 0.0 - top.state.v15[3]
    error_int = 0.0

    F = kp*error + kd*error_dot + ki*error_int

    τext = [0, 0, 0.0]
    Fext = [0, 0.0, F]
    # for joint in mechanism.joints
    #     if any([check_joint_name(joint, i, j) for i in 1:num_cells for j in 1:6]) 

    #         println(joint.name, get_joint_state(mechanism, joint))
    #     end
    # end
    set_external_force!(top, force=Fext, torque=τext)
    
end
mechanism = create_mechanism(;mechanism_dict...)
function load_mechanism_state!(mechanism, storage)
    for (i, body) in enumerate(mechanism.bodies)
        body.state.x1 = storage.x[i][1]
        body.state.q1 = storage.q[i][1]
        body.state.vsol = [szeros(eltype(storage.x[i][1][1]), 3) for i=1:2]
        body.state.ωsol = [szeros(eltype(storage.x[i][1][1]), 3) for i=1:2]
        body.state.x2 = storage.x[i][1]
        body.state.q2 = storage.q[i][1]
        body.state.v15 = storage.v[i][1]
        body.state.ω15 = storage.ω[i][1]
    end
end
load_mechanism_state!(mechanism, storage)
# Dojo.set_entries!(mechanism, reg=opts.reg) # compute the residual
mechanism.timestep = 0.005
Tf = 100*mechanism.timestep
timesteps = 0:mechanism.timestep:Tf
steps = length(timesteps)
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=false, reg=reg, max_iter=100)
simulate!(mechanism, 1:steps, storage, control!, record=true, opts=opts)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.01)

# plot position of the top plate over time
using Plots
plot(timesteps, ones(steps))
plot!(timesteps, [storage.x[end][i][3] for i in 1:steps])

# if !isdefined(Main, :collapsed_state)
# collapsed_state = load("media/kresling/kresling_2_cells_collapsed_state.jld2")["collapsed_state"]
# set_maximal_state!(mechanism, collapsed_state)
# else
# function control!(mechanism, t)
#     bottom = get_body(mechanism, Symbol("plate_1"))
#     middle = get_body(mechanism, Symbol("plate_2"))
#     if Dojo.norm(bottom.state.x2-middle.state.x2) < 4*plate_thickness
#         zero_velocities!(mechanism)
#     else
#         τext = [0, 0, 0.1]
#         Fext = [0, 0, 0.0]
#         # Do / Record stuff here 
#         for i in 1:num_cells+1
#             i % 2 == 1 ? set_external_force!(get_body(mechanism, Symbol("plate_$(i)")), force=Fext, torque=τext) : set_external_force!(get_body(mechanism, Symbol("plate_$(i)")), force=-Fext, torque=-2*τext) 
#         end
#     end
# end



collapsed_state = get_maximal_state(mechanism)

# end

set_maximal_state!(mechanism, collapsed_state)
zero_velocities!(mechanism)
println("here")

# middle = get_body(mechanism, Symbol("plate_2"))
# ωdes = [5.0, 0.0, 0.0]
# for body in bodies
#     vertex = body.state.x2-middle.state.x2
#     set_maximal_velocities!(body, v=Dojo.cross(vertex, ωdes), ω=ωdes)
# end
function control2!(mechanism, t)
    middle = get_body(mechanism, Symbol("plate_$(Int((num_cells)/2))"))
    if middle.state.ω15[1] < 5.0
        set_external_force!(middle, force=[0.0, 0.0, 0.0], torque=[0.5, 0.0, 0.0])

    else
        set_external_force!(middle, force=[0.0, 0.0, 0.0], torque=[0.0, 0.0, 0.0])
    end

end

# set_maximal_velocities!(middle, v=[0.0, 0.0, 0.0], ω=[1.0, 0.0, 0.0])
opts = SolverOptions(verbose=false, reg=reg, max_iter=100)
storage = simulate!(mechanism, 5.0, control2!, record=true, opts=opts)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.01)

using JLD2
save("media/kresling/kresling_$(num_cells)_cells_spin_open_sim.jld2", "storage", storage)
save("media/kresling/kresling_$(num_cells)_cells.jld2", "mechanism", mechanism)

# return mechanism, storage1, storage, vis, collapsed_state
# end
mechanism, storage_1, storage, vis, collapsed_state = run_all()

function control!(mechanism, t)
    bottom = get_body(mechanism, Symbol("plate_1"))
    top = get_body(mechanism, Symbol("plate_$(num_cells+1)"))
    middle = get_body(mechanism, Symbol("plate_2"))
    if Dojo.norm(bottom.state.x2-middle.state.x2) < 4*plate_thickness
        zero_velocities!(mechanism)
    else
        τext = [0, 0, 0.01]
        Fext = [0, 0, 0.0]
        # Do / Record stuff here 
        set_external_force!(bottom, force=Fext, torque=τext)
        set_external_force!(middle, force=-Fext, torque=-τext)
        set_external_force!(top, force=Fext, torque=τext)
    end
end
Tf = 1.0 
timesteps = 0:mechanism.timestep:Tf
steps = length(timesteps)
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=true, reg=reg, max_iter=100)
storage = simulate!(mechanism, Tf, control!, record=true, opts=opts)

vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=false, show_joint=false, joint_radius=0.05)

collapsed_state = get_maximal_state(mechanism)
filename = "kresling_2_cells_collapsed_state.jld2"
using JLD2
@save filename collapsed_state

# filename = "kresling_20_cells.jld2"
# @save filename mechanism

# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments
using JLD2
# function run_all()
# ### Parameters
# Link params
radius = 0.1
link_length = 1.0
diag_length = sqrt(2*link_length^2*(1 - cos(deg2rad(80))))
mass = 0.001

# Plate params
plate_radius = 1.0
plate_thickness = 0.1
plate_mass = 0.001

# Sim params
damper = 0.01
timestep = 0.005
reg = 1e-6
gravity = [0, 0, 0.0]

# System Params
num_cells = 10

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
        body = Cylinder(radius, link_length, mass, name=Symbol("link$(j)_cell$(i)"))
        push!(short_members[i], body)

        # long member
        diag1 = Cylinder(radius, diag_length/2, mass, name=Symbol("bot_diag$(j)_cell$(i)"))
        push!(long_members[i], diag1)

        diag2 = Cylinder(radius, diag_length/2, mass, name=Symbol("top_diag$(j)_cell$(i)"))
        push!(long_members[i], diag2)
    end
end

# ### Make joints
floating_joint = JointConstraint(Floating(origin, plates[1]), name=Symbol("floating_joint"))
push!(joints[1], floating_joint)

for i in 1:num_cells
    base = plates[i]
    top = plates[i+1]
    bot_start = -pi/3 #i % 2 == 1 ? 0 : -pi/3
    top_start = 0 #i % 2 == 1 ? -pi/3 : 0
    for j in 1:6
        # short member
        parent_vertex = [base.shape.rh[1]*cos(pi/3*j), base.shape.rh[1]*sin(pi/3*j), base.shape.rh[2]/2]
        child_vertex = [0, 0, -short_members[i][j].shape.rh[2]/2]

        joint = JointConstraint(Spherical(base, short_members[i][j]; 
                                                parent_vertex=parent_vertex, 
                                                child_vertex=child_vertex), 
                                                name=Symbol("joint_$(base.name)_to_$(short_members[i][j].name)"))
        push!(joints[i], joint)

        parent_vertex = [top.shape.rh[1]*cos(pi/3*j), top.shape.rh[1]*sin(pi/3*j), -top.shape.rh[2]/2]
        child_vertex = [0, 0, short_members[i][j].shape.rh[2]/2]

        joint = JointConstraint(Spherical(top, short_members[i][j]; 
                                                parent_vertex=parent_vertex, 
                                                child_vertex=child_vertex), 
                                                name=Symbol("joint_$(base.name)_to_$(short_members[i][j].name)"))
        push!(joints[i], joint)

        # long member bottom
        diag_bottom = long_members[i][2*j-1]
        bot_θ = bot_start + pi/3*(j-1)
        parent_vertex = [base.shape.rh[1]*cos(bot_θ), base.shape.rh[1]*sin(bot_θ), base.shape.rh[2]/2]
        child_vertex = [0, 0, -diag_bottom.shape.rh[2]/2]

        joint = JointConstraint(Spherical(base, diag_bottom; 
                                        parent_vertex=parent_vertex, 
                                        child_vertex=child_vertex), 
                                        name=Symbol("joint_$(base.name)_to_$(diag_bottom.name)"))
        push!(joints[i], joint)
        
        # long member top
        diag_top = long_members[i][2*j]
        top_θ = top_start + pi/3*(j-1)
        parent_vertex = [top.shape.rh[1]*cos(top_θ), top.shape.rh[1]*sin(top_θ), -top.shape.rh[2]/2]
        child_vertex = [0, 0, diag_top.shape.rh[2]/2]

        joint = JointConstraint(Spherical(top, diag_top; 
                                        parent_vertex=parent_vertex, 
                                        child_vertex=child_vertex), 
                                        name=Symbol("joint_$(base.name)_to_$(diag_top.name)"))
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

    bot_start = -pi/3 #i % 2 == 1 ? 0 : -pi/3
    top_start = 0 #i % 2 == 1 ? -pi/3 : 0

    for j in 1:6
        # short member
        set_maximal_configurations!(get_body(mechanism, Symbol("link$(j)_cell$(i)")), x=[plate_radius*cos(pi/3*j), plate_radius*sin(pi/3*j), link_length/2+(link_length+plate_thickness)*(i-1)])

        # long member
        bot_θ = bot_start + pi/3*(j-1)
        top_θ = top_start + pi/3*(j-1)

        set_maximal_configurations!(get_body(mechanism, Symbol("bot_diag$(j)_cell$(i)")), x=[plate_radius*cos(bot_θ), plate_radius*sin(bot_θ), diag_length/4+(link_length+plate_thickness)*(i-1)])

        set_maximal_configurations!(get_body(mechanism, Symbol("top_diag$(j)_cell$(i)")), x=[plate_radius*cos(top_θ), plate_radius*sin(top_θ), diag_length*3/4+(link_length+plate_thickness)*(i-1)])
    end
end
set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(num_cells+1)")), x=[0, 0, (link_length+plate_thickness)*(num_cells)])

plate_ids = [get_body(mechanism, Symbol("plate_$(i)")).id for i in 1:num_cells+1]
initialize_constraints!(mechanism, fixedids=plate_ids, regularization=1e-5, lineIter=10, newtonIter=300)
delete!(vis)
vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)

joints = JointConstraint{Float64}[deepcopy(mechanism.joints)...]


max_extension = sqrt(2*link_length^2*(1 - cos(deg2rad(120))))-diag_length
for i in 1:num_cells
    for j in 1:6
        for (id, joint) in enumerate(joints)
            if joint.name == Symbol("joint_bot_diag$(j)_cell$(i)_to_top_diag$(j)_cell$(i)")
                print("here")
                joints[id] = Dojo.add_limits(mechanism, joint,
                tra_limits=[Dojo.SVector{1}(0.0), Dojo.SVector{1}(max_extension)])
            end
        end
    end
end

mechanism = Mechanism(mechanism.origin, mechanism.bodies, joints; gravity, timestep)
        # joints = set_limits(mechanism, joint_limits)
set_dampers!(mechanism.joints, damper)
# if isdefined(Main, :vis)
#     # If it exists, delete it
#     delete!(vis)
# else
#     # If it doesn't exist, initialize it as a Visualizer
#     vis = Visualizer()
# end
# vis = Visualizer()
# vis = visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, joint_radius=0.05)
# if !isdefined(Main, :collapsed_state)
# collapsed_state = load("media/kresling/kresling_2_cells_collapsed_state.jld2")["collapsed_state"]
# set_maximal_state!(mechanism, collapsed_state)
# else
out = load("kresling_10_cells_collapsed_sim.jld2")

mechanism = out["mechanism"]
storage1 = out["storage1"]
storage = out["storage"]

collapsed_state = load("kresling_10_cells_collapsed_state.jld2")["collapsed_state"]
off = 0
for body in mechanism.bodies
    # x2, v15, q2, ω15 = Dojo.unpack_data(collapsed_state[Dojo.SUnitRange(off+1,end)]); off += 13
    x2 = collapsed_state[off+1:off+3]
    v15 = collapsed_state[off+4:off+6]
    q2 = collapsed_state[off+7:off+10]
    ω15 = collapsed_state[off+11:off+13]
    off += 13
    q2 = Quaternion(q2...)
    body.state.v15 = v15
    body.state.ω15 = ω15
    body.state.x2 = x2
    body.state.q2 = q2
end
Dojo.initialize_state!(mechanism) # set x1, q1 and zeroes out JF2 Jτ2

for body in mechanism.bodies
    Dojo.set_velocity_solution!(body)
end

# set_maximal_state!(mechanism, collapsed_state)
# z = p1.state.x1[3]
# for i in 1:num_cells+1
#     set_maximal_configurations!(get_body(mechanism, Symbol("plate_$(i)")), x=[0, 0, z+4*plate_thickness*(i-1)])
# end
# vis = Visualizer()

plate_ids = [get_body(mechanism, Symbol("plate_$(i)")).id for i in 1:num_cells+1]



# for i in 10:-1:1
   
#     initialize_constraints!(mechanism, fixedids=[plate_ids[1], plate_ids[end]], regularization=1.0^i, lineIter=10, newtonIter=300)
# end

for _ in 1:3
    c = 0
    for body in mechanism.bodies
        if body.id in plate_ids
            set_maximal_configurations!(body, x=[0, 0, body.state.x2[3]- 0.01*c], q=body.state.q2*Dojo.RotZ(deg2rad(1.0*c)))
            c +=1
        end
    end
    initialize_constraints!(mechanism, fixedids=plate_ids, regularization=1e-6, lineIter=10, newtonIter=50, debug=true)

    visualize(mechanism; vis = vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.03)
end
collapsed_state = get_maximal_state(mechanism)
initialize_constraints!(mechanism, fixedids=plate_ids, regularization=1e-10, lineIter=10, newtonIter=100, debug=true)
vis = Visualizer()
visualize(mechanism; vis = vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.03)
zero_velocities!(mechanism)
set_dampers!(mechanism.joints, 1.0)


for body in mechanism.bodies
    body.state.d -= body.state.d
    body.state.D -=  body.state.D
    body.state.Fext -= body.state.Fext
    body.state.τext -= body.state.τext
    body.state.vsol[1] -= body.state.vsol[1] 
    body.state.vsol[2] -= body.state.vsol[2]
    body.state.ωsol[1] -= body.state.ωsol[1]
    body.state.ωsol[2] -= body.state.ωsol[2]
end



mechanism.timestep= 0.001
Tf = mechanism.timestep
timesteps = 0:mechanism.timestep:Tf
steps = length(timesteps)
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=true, reg=reg, max_iter=50)
simulate!(mechanism, 1:steps, storage, record=true, opts=opts)

visualize(mechanism, storage; vis = vis, visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.03)


function control!(mechanism, t)
    bottom = get_body(mechanism, Symbol("plate_10"))
    middle = get_body(mechanism, Symbol("plate_9"))
    # if Dojo.norm(bottom.state.x2-middle.state.x2) < 4*plate_thickness
    #     zero_velocities!(mechanism)
    # else
    τext = [0, 0, 0.01]
    Fext = [0, 0.0, 0.0]
    # Do / Record stuff here 
    for i in 1:num_cells+1
        if i > num_cells/2
            set_external_force!(get_body(mechanism, Symbol("plate_$(num_cells+1)")), force=-Fext, torque=τext) 
        else
            set_external_force!(get_body(mechanism, Symbol("plate_1")), force=Fext, torque=-τext) 
        end
    end

        # for i in 2:num_cells+1
            # set_external_force!(get_body(mechanism, Symbol("plate_$(i)")), force=-Fext, torque=τext)
            # i % 2 == 1 ? set_external_force!(get_body(mechanism, Symbol("plate_$(i)")), force=Fext, torque=τext) : set_external_force!(get_body(mechanism, Symbol("plate_$(i)")), force=-Fext, torque=-2*τext) 
        # end
    # end
end
Tf = 1.0 
timesteps = 0:mechanism.timestep:Tf
steps = length(timesteps)
storage = Storage(steps, length(mechanism.bodies))
opts = SolverOptions(verbose=false, reg=reg, max_iter=100)
simulate!(mechanism, 1:steps, storage, control!, record=true, opts=opts)
@save "kresling_10_cells_collapsed_state.jld2" collapsed_state
@save "kresling_10_cells_semi_closed.jld2" mechanism storage1 
frame_id = 1

for (body, x, q, ω, v) in zip(mechanism.bodies, storage.x, storage.q, storage.ω, storage.v)
    body.state.x1 = x[1]
    body.state.x2 = x[1]
    body.state.q1 = q[1]
    body.state.q2 = q[1]
    body.state.ω15 = ω[1]
    body.state.v15 = v[1]
    body.state.d -= body.state.d
    body.state.D -=  body.state.D
    body.state.Fext -= body.state.Fext
    body.state.τext -= body.state.τext
    body.state.vsol[1] -= body.state.vsol[1] 
    body.state.vsol[2] -= body.state.vsol[2]
    body.state.ωsol[1] -= body.state.ωsol[1]
    body.state.ωsol[2] -= body.state.ωsol[2]

end
zero_velocities!(mechanism)
delete!(vis)
visualize(mechanism; visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.03)

storage

collapsed_state = get_maximal_state(mechanism)
using JLD2
@save "kresling_10_cells_collapsed_sim.jld2" mechanism storage storage1 
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
        set_external_force!(middle, force=[0.0, 0.0, 0.0], torque=[1.0, 0.0, 0.0])

    else
        set_external_force!(middle, force=[0.0, 0.0, 0.0], torque=[0.0, 0.0, 0.0])
    end

end
# set_maximal_state!(mechanism, collapsed_state)
# set_maximal_velocities!(middle, v=[0.0, 0.0, 0.0], ω=[1.0, 0.0, 0.0])
opts = SolverOptions(verbose=false, reg=reg, max_iter=100)
storage = simulate!(mechanism, 10.0, control2!, record=true, opts=opts)
@save "kresling_10_cells_spining_sim.jld2" mechanism storage
visualize(mechanism, storage; visualize_floor=false, show_frame=false, show_joint=true, joint_radius=0.03)
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

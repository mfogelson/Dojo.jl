using Dojo
include("../../ClosedChainEnvironments/src/utils/parse_json.jl")

# Import the JSON file describing the mechanism
filename = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/10_ZeroG_Flight/ZeroG Sim Data/Translational Scissor/2025_05_14/four_link_short_scissor v12_link_dict.json"

mechanism, contact = parse_json(filename, slop=0.0, gravity=zeros(3), damper=0.00005,  timestep=0.01)#, translation_offset, rotation_offset, true); rot_joint_limits_even=[[-2.74], [2.74]],rot_joint_limits_odd=[[-2.74], [2.74]],

for joint in mechanism.joints
    println("Joint: ", joint.name)
    println("Angle: ", Dojo.minimal_coordinates(mechanism, joint))
end
# Set up the visualization of the mechanism
vis = Visualizer()
delete!(vis)
visualize(mechanism, vis=vis, visualize_floor=false, show_frame=false, show_joint=true, show_contact=true, joint_radius=0.005)

# Add initial conditions
ω = [0.0, 0.0, 2π * 1.0]  # 10 Hz spin around z
r_com = [0.0, 0.0, 0.0]  # Center of mass
positions = [body.state.x2 for body in mechanism.bodies]  # Positions of the bodies
v_linear = [Dojo.cross(ω, r_i - r_com) for r_i in positions]

# Set the initial state of the mechanism
for (i, body) in enumerate(mechanism.bodies)
    body.state.ω15 = Dojo.vector_rotate(ω, body.state.q2')
    body.state.v15 = Dojo.vector_rotate(v_linear[i], body.state.q2')
end

# Run the simulation
t = 0.0
dt = 0.01
t_max = 3.0
steps = Int(t_max / dt)
solver = Dojo.mehrotra_svd!
opts = SolverOptions(rtol=1e-4, btol=1e-4, undercut=1.0, reg=1e-10,verbose=false, svd_threshold=1e-6)
storage = Storage(steps, length(mechanism.bodies))
function control!(mechanism, t)
    for joint in mechanism.joints
    println("Joint: ", joint.name)
    println("Angle: ", Dojo.minimal_coordinates(mechanism, joint))
    end
end
simulate!(mechanism, 1:steps, storage, control!, record=true , opts=opts, abort_upon_failure=true, solver=solver)

function set_background_transparent!(vis::Visualizer)
    MeshCat.exec(vis, """
        viewer.renderer.setClearColor(0xffffff, 0);  // white with alpha 0
        viewer.renderer.alpha = true;
        viewer.renderer.clearAlpha = 0;
        viewer.renderer.setClearAlpha(0);
    """)
end
set_background!(vis)
vis = visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true, show_joint=true, show_contact=true, joint_radius=0.005)

for joint in mechanism.joints
    println("Joint: ", joint.name)
    println("Angle: ", Dojo.minimal_coordinates(mechanism, joint))
end

include("../optitrack_to_struct.jl")
csv_file = "/Users/mitchfogelson/Projects/Research_Projects/ZeroG_Flight/gopro_calibration/data/OptiTrack/session_2025_05_03/Take 2025-05-03 12.24.11 PM translational_scissor.csv"

visualize_optitrack_file(csv_file)
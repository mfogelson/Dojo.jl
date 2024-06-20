using Dojo
using CSV
using DataFrames

# Create the mechanism
RADIUS = 1.0 # Maximum Diameter of the rocket fairing
THICKNESS = 0.025 # Maximum Diameter of the rocket fairing
MASS = 1.0

origin = Origin()
plates = Body{Float64}[]
joints = JointConstraint{Float64}[]
gravity = [0.0, 0.0, 0.0]

base = Cylinder(RADIUS, THICKNESS, MASS, name=Symbol("base_plate"), color=RGBA(1.0, 0, 0, 1.0))
push!(plates, base)

middle = Cylinder(RADIUS, THICKNESS, MASS, name=Symbol("middle_plate"), color=RGBA(0, 1.0, 0, 1.0))
push!(plates, middle)

top = Cylinder(RADIUS, THICKNESS, MASS, name=Symbol("top_plate"), color=RGBA(0, 0, 1.0, 1.0))
push!(plates, top)

fixed_joint = JointConstraint(Fixed(origin, plates[1]), name=Symbol("fixed_joint"))
push!(joints, fixed_joint)

mechanism = Mechanism(origin, plates, joints; gravity)

# Load CSV data related to the position and orientation of the plates
# Julia load csv data 
file_path = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/01_HERDS_Fogelson_Thomas_Falcon_Lipton_Manchester/Kresling_Deploy_Run_01/natnet_ros-HexagonMiddle-pose.csv"

# Read the CSV file into a DataFrame
df = CSV.File(file_path) |> DataFrame
position_middle = df[!, ["pose.position.x", "pose.position.y", "pose.position.z"]]
orientation_middle = df[!, ["pose.orientation.w", "pose.orientation.x", "pose.orientation.y", "pose.orientation.z"]]

file_path = "/Users/mitchfogelson/Library/CloudStorage/Box-Box/00_Mitch Fogelson/00_Research/00_Niac_Space_Structures/01_HERDS_Fogelson_Thomas_Falcon_Lipton_Manchester/Kresling_Deploy_Run_01/natnet_ros-HexagonTop-pose.csv"

# Read the CSV file into a DataFrame
df = CSV.File(file_path) |> DataFrame
position_top = df[!, ["pose.position.x", "pose.position.y", "pose.position.z"]]
orientation_top = df[!, ["pose.orientation.w", "pose.orientation.x", "pose.orientation.y", "pose.orientation.z"]]
# Loop through the data and visualize the mechanism
vis = Visualizer()
idx = 15500
set_maximal_configurations!(get_body(mechanism, Symbol("base_plate")), x=[Vector(position_middle[idx,1:2]);0.0], q=Quaternion(1.0, 0.0, 0.0, 0.0))
set_maximal_configurations!(get_body(mechanism, Symbol("middle_plate")), x=Vector(position_middle[idx,:]), q=Quaternion(orientation_middle[idx,1], orientation_middle[idx,2], orientation_middle[idx,3], orientation_middle[idx,4]))
set_maximal_configurations!(get_body(mechanism, Symbol("top_plate")), x=Vector(position_top[idx,:]), q=Quaternion(orientation_top[idx,1], orientation_top[idx,2], orientation_top[idx,3], orientation_top[idx,4]))
delete!(vis)
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)

count = 0
start_ind = 13500
end_ind = 15500
for (x_middle, x_top, q_middle, q_top) in zip(eachrow(position_middle[start_ind:end_ind, :]), eachrow(position_top[start_ind:end_ind, :]), eachrow(orientation_middle[start_ind:end_ind, :]), eachrow(orientation_top[start_ind:end_ind, :]))
    if count < 20
        count += 1
        continue
    end
    set_maximal_configurations!(get_body(mechanism, Symbol("middle_plate")), x=Vector(x_middle), q= Quaternion(q_middle[1], q_middle[2], q_middle[3], q_middle[4]))
    set_maximal_configurations!(get_body(mechanism, Symbol("top_plate")), x=Vector(x_top), q= Quaternion(q_top[1], q_top[2], q_top[3], q_top[4]))
    visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)
    sleep(0.1)
    count = 0
end

position_middle[start_ind, :]
position_middle[end_ind, :]
position_top[start_ind, :]
position_top[end, :]
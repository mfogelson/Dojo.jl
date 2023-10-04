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
set_maximal_configurations!(get_body(mechanism, Symbol("base_plate")), x=[0.0, 0.0, 0.0], q=Quaternion([1.0, 0.0, 0.0, 0.0]))
set_maximal_configurations!(get_body(mechanism, Symbol("middle_plate")), x=Vector(position_middle[1,:]), q=Quaternion(Vector(orientation_middle[1,:])))
set_maximal_configurations!(get_body(mechanism, Symbol("top_plate")), x=Vector(position_top[1,:]), q=Quaternion(Vector(orientation_top[1,:])))
delete!(vis)
visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)

count = 0
for (x_middle, x_top, q_middle, q_top) in zip(eachrow(position_middle), eachrow(position_top), eachrow(orientation_middle), eachrow(orientation_top))
    if count < 20
        count += 1
        continue
    end
    set_maximal_configurations!(get_body(mechanism, Symbol("middle_plate")), x=Vector(x_middle), q=Quaternion(Vector(q_middle)))
    set_maximal_configurations!(get_body(mechanism, Symbol("top_plate")), x=Vector(x_top), q=Quaternion(Vector(q_top)))
    visualize(mechanism; vis=vis, visualize_floor=false, show_frame=true, show_joint=false, joint_radius=0.05)
    sleep(0.1)
    count = 0
end
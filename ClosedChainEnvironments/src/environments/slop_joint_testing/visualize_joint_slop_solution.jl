using Dojo
using JLD2
using Plots
plotly()
mechanism, storage = load("paper_data/Scissor_jamming/09_25_2024_23_cell_0.01_damp_0.001_slop_scissor_mechanism.jld2", "mechanism", "storage");

body1 = mechanism.bodies[1]

Dojo.constraint(mechanism, body1)
Dojo.constraint_jacobian_configuration(mechanism, body1)

vis = Visualizer()
visualize(mechanism, storage; vis=vis, visualize_floor=false, show_frame=true)

# visualize joint and constraint information
p = plot(aspect_ratio=:equal, xlabel="x", ylabel="y")

for joint in mechanism.joints
    if joint.type != :PlanarAxis
        continue
    end
    
    parent_body = get_body(mechanism, joint.parent_id)
    parent_joint_pos = parent_body.state.x2 + Dojo.vector_rotate(joint.translational.vertices[1], parent_body.state.q2)

    parent_limit_pp = parent_joint_pos + Dojo.vector_rotate([0.0, joint.translational.joint_limits[2][1], joint.translational.joint_limits[2][2]], parent_body.state.q2)
    parent_limit_pm = parent_joint_pos + Dojo.vector_rotate([0.0, joint.translational.joint_limits[2][1], joint.translational.joint_limits[1][2]], parent_body.state.q2)
    parent_limit_mp = parent_joint_pos + Dojo.vector_rotate([0.0, joint.translational.joint_limits[1][1], joint.translational.joint_limits[2][2]], parent_body.state.q2)
    parent_limit_mm = parent_joint_pos + Dojo.vector_rotate([0.0, joint.translational.joint_limits[1][1], joint.translational.joint_limits[1][2]], parent_body.state.q2)

    scatter!([parent_joint_pos[2]], [parent_joint_pos[3]], color="red", label="") #, label="Joint Position", color="red")
    plot!([parent_limit_pp[2], parent_limit_pm[2]], [parent_limit_pp[3], parent_limit_pm[3]], color="blue", label="") #, label="Joint Limits", color="blue")
    plot!([parent_limit_pm[2], parent_limit_mm[2]], [parent_limit_pm[3], parent_limit_mm[3]], color="blue", label="") #, label="Joint Limits", color="blue")
    plot!([parent_limit_mm[2], parent_limit_mp[2]], [parent_limit_mm[3], parent_limit_mp[3]], color="blue", label="") #, label="Joint Limits", color="blue")
    plot!([parent_limit_mp[2], parent_limit_pp[2]], [parent_limit_mp[3], parent_limit_pp[3]], color="blue", label="") #, label="Joint Limits", color="blue")

    parent_forces = Dojo.impulse_map(mechanism, joint, parent_body)*joint.impulses[2]
    parent_forces = parent_joint_pos + Dojo.vector_rotate(parent_forces[1:3], parent_body.state.q2)

    plot!([parent_joint_pos[2], parent_forces[2]], [parent_joint_pos[3], parent_forces[3]], color="black", label="", arrow=true) #, label="Joint Forces", color="black")


    child_body = get_body(mechanism, joint.child_id)
    child_joint_pos = child_body.state.x2 + Dojo.vector_rotate(joint.translational.vertices[2], child_body.state.q2)

    scatter!([child_joint_pos[2]], [child_joint_pos[3]], color="green", label="") #, label="Joint Position")

    child_forces = Dojo.impulse_map(mechanism, joint, child_body)*joint.impulses[2]
    child_forces = child_joint_pos + Dojo.vector_rotate(child_forces[1:3], child_body.state.q2)
    plot!([child_joint_pos[2], child_forces[2]], [child_joint_pos[3], child_forces[3]], color="black", label="", arrow=true) #, label="Joint Forces", color="black")
end
plot!(xlims=(-0.07, 0.07), ylims=(0.00, 0.3))

savefig(p, "test.html")

@elapsed out = step!(mechanism, get_maximal_state(mechanism), zeros(Dojo.input_dimension(mechanism)))

@time Dojo.mehrotra_niave!(mechanism);
using LinearAlgebra
@time A = Dojo.full_matrix(mechanism.system);
@time b = Dojo.full_vector(mechanism.system);
@time F = svd(A, full=true, alg=LinearAlgebra.QRIteration()); #, full=true, alg=LinearAlgebra.QRIteration());
# end
# F = svd(A) #svd(A, full=true, alg=LinearAlgebra.QRIteration())
rank = sum(F.S .> opts.svd_threshold)
V1 = @view F.V[:,1:rank]
S1 = @view F.S[1:rank]
U1 = @view F.U[:,1:rank]
# U1, S1, V1 = tsvd(A, k=140)

out = V1*Diagonal(1.0 ./ S1)*U1'*b

start = 0
for i in eachindex(mechanism.system.vector_entries)
    mechanism.system.vector_entries[i].value = out[start + 1: start+size(mechanism.system.vector_entries[i].value, 1)]
    start += size(mechanism.system.vector_entries[i].value, 1)
end
using Dojo
using JLD2
using Dates

include("scissor_mechanism.jl")
include("controllers.jl")

"""
Run experiments with different solvers and number of cells.

Parameters:
- solvers: List of solvers to test
- max_cells: Maximum number of cells to test
- steps: Number of simulation steps
- slop: Amount of play in the joints
- initial_angle: Initial angle of the mechanism
"""
function run_experiments(solvers, max_cells, steps, slop, initial_angle)
    for solver in solvers
        for cells in 1:max_cells
            println("Initialized Mechanism with $cells cells")

            mechanism = get_scissor_mechanism(num_sets=cells, initial_angle=initial_angle, slop=slop)

            println("Starting Simulation with $solver")
            storage = Storage(steps, length(mechanism.bodies))
            
            simulate!(mechanism, 1:steps, storage, angle_controller!, 
                      record=true, 
                      opts=SolverOptions(rtol=1e-7, btol=1e-4, reg=1e-10, verbose=false, svd_threshold=1e-7),
                      abort_upon_failure=true,
                      solver=solver)
            
            println("Simulation Done")

            datetime = now()
            println("Saving Data")
            save("paper_data/Scissor_Sweep/$(solver)/scissor_cells_$(cells)_slope_$(slop)_theta0_$(round(initial_angle, digits=2))_$datetime.jld2", 
                 "mechanism", mechanism, "storage", storage)
        end
    end
end

# Example usage
solvers = [Dojo.mehrotra!, Dojo.mehrotra_svd!, Dojo.mehrotra_niave!]
max_cells = 20
steps = 500
slop = 0.0
initial_angle = -pi*9/10
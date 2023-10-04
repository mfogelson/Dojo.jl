using Dojo
using JuMP
using Ipopt
using Juniper
using HiGHS
using CSV
using DataFrames
using Printf
# Function to update DataFrame based on key-value pairs
# function update_dataframe!(df::DataFrame, search_col::Symbol, update_col::Symbol, changes::Dict)
#     for (key, value) in changes
#         filter_row_indices = findall(isequal(key), df[!, search_col])
#         df[filter_row_indices, update_col] .= string(value)
#     end
# end

function update_dataframe!(df::DataFrame, search_col::Symbol, update_col::Symbol, changes::Dict)
    for (key, value) in changes
        filter_row_indices = findall(isequal(key), df[!, search_col])
        
        if isempty(filter_row_indices)  # If no rows are found for the key
            # new_row_values = [col == search_col ? key : col == update_col ? string(value) : missing for col in names(df)]
            # new_row = NamedTuple{Tuple(names(df)...)}(Tuple(new_row_values))
            append!(df, Dict(:name=>key, :units=>" ", :value=>string(value), :Column4=>missing))
        else
            df[filter_row_indices, update_col] .= string(value)
        end
    end
end


function optimize_configuration(; VEHICAL_NAME, DIAMETER, MASS, INITIAL_HEIGHT, EXPANDED_HEIGHT, STATES=5, DENSITY=2710.0)
    # model = Model(Ipopt.Optimizer)
    ipopt = optimizer_with_attributes(Ipopt.Optimizer, "print_level" => 0)
    highs = optimizer_with_attributes(HiGHS.Optimizer, "output_flag" => false)
    model = Model(
        optimizer_with_attributes(
            Juniper.Optimizer,
            "nl_solver" => ipopt,
            "mip_solver" => highs,
            "time_limit" => 60.0,
        ),
    )
    set_silent(model)
    # Constraint Constants 
    # DIAMETER = 8.4 # Maximum Diameter of the rocket fairing
    # MASS = 10.0 # Maximimum Mass of the payload
    # INITIAL_HEIGHT = 27.8 # Maximum Height of the rocket fairing
    # EXPANDED_HEIGHT = 1000.0 # Final expanded height
    # EXPANSION_RATIO = EXPANDED_HEIGHT/INITIAL_HEIGHT
    # STATES = 2

    #######################################################################
    # Decision variables folding scissor 
    @variable(model, 0.01 <= l1 <= 10.0) # Half of one link
    @variable(model, 0.01 <= l2 <= 10.0) # The extension 
    @variable(model, 0.01 <= l3 <= 10.0) # Half of one link
    @variable(model, thickness >= 0.001)
    @variable(model,  0.0 <= α[1:STATES] <= pi) # For the big members
    @variable(model, 0.0 <= β[1:STATES] <= pi) # for the small Members 
    @variable(model, n>=1, Int) # number of members 

    # Decision variables for the kresling
    # @variable(model, 0.0 <= r <= DIAMETER/2)
    # @variable(model, -pi/3 <= phi[1:STATES] <= pi/3)
    # @variable(model, h[1:STATES] >= 0.002)

    # Decision variables super structure
    # @variable(model, m>=1, Int) # number of cells

    #######################################################################
    # Useful Expressions folding scissor
    a = @NLexpression(model, [i=1:STATES], (l2+l3)*sqrt(2*(1-cos(α[i])))) # Width 
    b = @NLexpression(model, [i=1:STATES], l1*sqrt(2*(1-cos(β[i])))) # Edge
    c = @NLexpression(model, [i=1:STATES], 2*l1*sqrt(2*(1-cos(pi-β[i])))) # Extension
    d1 = @NLexpression(model, [i=1:STATES], sqrt((l2+l3)^2+l3^2-2*(l3*(l2+l3))*cos(pi-α[i])))
    d2 = @NLexpression(model, [i=1:STATES], sqrt((l3)^2+l2^2-2*(l3*(l2))*cos(α[i])))
    θ = @NLexpression(model, [i=1:STATES], (a[i]^2-2*b[i]^2)/(-2*b[i]^2)) # cos theta
    scissor_length = @NLexpression(model, [i=1:STATES],  c[i]*n) # Total length of the scissor
    # contact_l3 = @NLexpression(model, (thickness/l3))
    # contact_l1 = @NLexpression(model, (thickness/l1))

    # Constraints folding scissor
    @NLconstraint(model, [i=1:STATES], a[i] <= 2*b[i]) # Pop triangle equation
    # Extension triangle constriants
    @NLconstraint(model, [i=1:STATES],c[i] <= l2 + d1[i])
    @NLconstraint(model,[i=1:STATES],c[i] + l2 >= d1[i])

    # Trapazoid constraints 
    @NLconstraint(model, [i=1:STATES], d1[i]^2+d2[i]^2-c[i]^2-l3^2-2*(l2*(l2+l3))==0)

    # Inequality Constraints
    @NLconstraint(model,[i=1:STATES], a[i] >= 0.001)
    @NLconstraint(model,[i=1:STATES], b[i] >= 0.001)
    @NLconstraint(model,[i=1:STATES], c[i] >= 0.001)

    # Angle constraints
    @NLconstraint(model, [i = 1:STATES], l3/(sqrt(l3^2+thickness^2)) - cos(α[i]/2) >= 0.0)
    @NLconstraint(model, [i = 1:STATES], l3/(sqrt(l3^2+thickness^2)) - cos((pi-α[i])/2) >= 0.0)

    @NLconstraint(model, [i = 1:STATES], l1/(sqrt(l1^2+thickness^2)) - cos(β[i]/2) >= 0.0)
    @NLconstraint(model, [i = 1:STATES], l1/(sqrt(l1^2+thickness^2)) - cos((pi-β[i])/2) >= 0.0)

    @constraint(model, [i = 1:STATES-1], α[i+1] <= α[i])
    @constraint(model, [i = 1:STATES-1], β[i+1] <= β[i])

    #######################################################################
    # Useful Expressions for the kresling
    # diameter = @NLexpression(model, 2*a[10] + 2*r) # Diameter of the structure
    # height = @NLexpression(model, [i=1:10], c[i]*sin(γ[i])) # Height of the structure
    # lBC = @NLexpression(model, [i=1:STATES], sqrt(h[i]^2 - 2*r^2*cos(phi[i])+2*r^2))
    # lAC = @NLexpression(model, [i=1:STATES], sqrt(h[i]^2 - 2*r^2*cos(2*pi/6 + phi[i])+2*r^2))

    # Kresling Constraints 
    # @NLconstraint(model, scissor_length[1] == r)
    # @NLconstraint(model, scissor_length[1] == lBC[1])
    # @variable(model, 0.0<=t<=1.0) # intersection var 
    # @variable(model, 0.0<=s<= 1.0) # intersection var
    # x constraint
    # @NLconstraint(model, r + t*(r*cos(pi/3+phi[1])-r) == r*cos(pi/3) + t*(r*cos(2pi/3+phi[1])-r*cos(pi/3)))
    # # y constraint
    # @NLconstraint(model, t*(-r*sin(pi/3+phi[1])) == -r*sin(pi/3) + t*(-r*sin(2pi/3+phi[1])+r*sin(pi/3)))
    # z constraint
    # [r + t*(r*cos(pi/3+phi[1])-r), t*(-r*sin(pi/3+phi[1])), t*h[1]]-[r*cos(pi/3) + t*(r*cos(2pi/3+phi[1])-r*cos(pi/3)), -r*sin(pi/3) + t*(-r*sin(2pi/3+phi[1])+r*sin(pi/3)), t*h[1]]
    # @NLconstraint(model, sqrt(sum(((r + t*(r*cos(pi/3+phi[1])-r))-r*cos(pi/3) - s*(r*cos(2pi/3+phi[1])-r*cos(pi/3)))^2+(t*(-r*sin(pi/3+phi[1]))-(-r*sin(pi/3) + s*(-r*sin(2pi/3+phi[1])+r*sin(pi/3))))^2) + (t*h[1]-s*h[1])) == 4*thickness)
    # @NLconstraint(model, phi[1] == deg2rad(55))
    # @NLconstraint(model, h[1] - 4*thickness >= 0.0)
    # @NLconstraint(model, scissor_length[STATES] == lBC[STATES])
    # @NLconstraint(model, scissor_length[STATES] == lAC[STATES])

    #######################################################################
    # Expansion ratio constraint 
    # @NLconstraint(model, 2*r + 2*a[1] <= DIAMETER)
    volume = @NLexpression(model, (2*thickness)^2*((2*l1)*8*n + 2*(2*l3+l2)*n))
    mass = @NLexpression(model, volume*DENSITY)
    @NLconstraint(model, mass <= MASS)
    @NLconstraint(model, a[1] <= DIAMETER)
    @NLconstraint(model, scissor_length[1] <= INITIAL_HEIGHT)
    @NLconstraint(model, scissor_length[STATES] >= EXPANDED_HEIGHT)
    @NLconstraint(model, n*10 <= 750000)
    @NLconstraint(model, θ[1] == cos(pi*0.95))
    @NLconstraint(model, θ[STATES] == cos(pi/3))

    # @NLconstraint(model, a[1] <= r)
    # @NLconstraint(model, phi[1] >= 0.0)
    # @NLconstraint(model, h[1] >= 4.0*thickness)
    # @NLconstraint(model, h[1]*m <= INITIAL_HEIGHT)
    # @NLconstraint(model, h[STATES]*m >= EXPANDED_HEIGHT)

    #######################################################################
    scale = 2.0*1000.0
    @NLobjective(model, Min, ((l1+l2+l3)*scale)/((thickness*scale)^4))
    optimize!(model)

    #######################################################################
    # Define changes: HERDS_global_params.csv
    # df = CSV.File("/Users/mitchfogelson/Downloads/HERDS_global_params.csv", header=["name", "units", "value"]) |> DataFrame

    # changes = Dict("height" => value(h[1])*1000.0, "phi" => rad2deg(value(phi[1])), "radius" => value(r)*1000.0, "thickness" => value(thickness)*1000.0, "lAB" => value(r)*1000.0, "lBC" => value(lBC[1])*1000.0, "lAC" => value(lAC[1])*1000.0)

    # # Apply the changes
    # update_dataframe!(df, :name, :value, changes)

    # Save the modified DataFrame to a new CSV file
    # CSV.write("/Users/mitchfogelson/Downloads/$(VEHICAL_NAME)_HERDS_initialState.csv", df, writeheader=false)

    # changes = Dict("height" => value(h[STATES])*1000.0, "phi" => rad2deg(value(phi[STATES])), "radius" => value(r)*1000.0, "thickness" => value(thickness)*1000.0, "lAB" => value(r)*1000.0, "lBC" => value(lBC[STATES])*1000.0, "lAC" => value(lAC[STATES])*1000.0)

    # # Apply the changes
    # update_dataframe!(df, :name, :value, changes)

    # Save the modified DataFrame to a new CSV file
    # CSV.write("/Users/mitchfogelson/Downloads/$(VEHICAL_NAME)_HERDS_finalState.csv", df, writeheader=false)

    # Define changes: PET_global_params.csv
    df = CSV.File("/Users/mitchfogelson/Downloads/PET_global_params.csv", header=["name", "units", "value"]) |> DataFrame

    changes = Dict("l1" => value(l1), "l2" => value(l2), "l3" => value(l3), "l2l3" => (value(l3)+value(l2)), "thickness" => value(thickness)*1000.0, "alpha" => (value(α[1])), "beta" => (value(β[1])), "n"=>value(n))

    # Apply the changes
    update_dataframe!(df, :name, :value, changes)

    # Save the modified DataFrame to a new CSV file
    CSV.write("/Users/mitchfogelson/Downloads/$(VEHICAL_NAME)_PET_only_initialState_mass.csv", df, writeheader=false)

    changes = Dict("l1" => value(l1), "l2" => value(l2), "l3" => value(l3), "l2l3" => (value(l3)+value(l2)), "thickness" => value(thickness)*1000.0, "alpha" => (value(α[STATES])), "beta" => (value(β[STATES])), "n"=>value(n))

    # Apply the changes
    update_dataframe!(df, :name, :value, changes)

    # Save the modified DataFrame to a new CSV file
    CSV.write("/Users/mitchfogelson/Downloads/$(VEHICAL_NAME)_PET_only_finalState_mass.csv", df, writeheader=false)

    # TODO: Fix this Save model to file 
    # write_to_file(model, "model_$VEHICAL_NAME.mof.json")

    keys = [:l1, :l2, :l3, :thickness, :α, :β, :n]
    for key in keys
        println(round(value.(model.obj_dict[key])[1], digits=3))
        # if length(model.obj_dict[key]) == 1
            
        # else
        #     println(value.(model.obj_dict[key]))
        # end
    end    
    println(round(value(a[1]), digits=2))
    formatted_string = @sprintf("%.2e", value(scissor_length[1]))
    println(formatted_string)
    formatted_string = @sprintf("%.2e", value(scissor_length[STATES]))
    println(formatted_string)
    formatted_string = @sprintf("%.2e", value(mass))
    println(formatted_string)

    println("Objective value: ", @sprintf("%.2e", objective_value(model)))

end

input_params_falcon = Dict(:VEHICAL_NAME => "falcon", 
                    :DIAMETER => 3.4, 
                    :MASS => 10.0*1000.0, 
                    :INITIAL_HEIGHT => 9.7, 
                    :EXPANDED_HEIGHT => 1000.0, 
                    :STATES => 2)

input_params_starship = Dict(:VEHICAL_NAME => "starship", 
                    :DIAMETER => 4.4, 
                    :MASS => 21.0*1000.0, 
                    :INITIAL_HEIGHT => 17.2, 
                    :EXPANDED_HEIGHT => 1000.0, 
                    :STATES => 2)

input_params_starship_ext = Dict(:VEHICAL_NAME => "starship_ext", 
                    :DIAMETER => 4.4, 
                    :MASS => 21.0*1000.0, 
                    :INITIAL_HEIGHT => 22.0, 
                    :EXPANDED_HEIGHT => 1000.0, 
                    :STATES => 2)

input_params_sls_b1 = Dict(:VEHICAL_NAME => "sls_b1", 
                    :DIAMETER => 8.4, 
                    :MASS => 42.0*1000.0, 
                    :INITIAL_HEIGHT => 19.0, 
                    :EXPANDED_HEIGHT => 1000.0, 
                    :STATES => 2)

input_params_sls_b2 = Dict(:VEHICAL_NAME => "sls_b2", 
                    :DIAMETER => 8.4, 
                    :MASS => 46.0*1000.0, 
                    :INITIAL_HEIGHT => 27.8, 
                    :EXPANDED_HEIGHT => 1000.0, 
                    :STATES => 2)

for input_params in [input_params_falcon, input_params_starship, input_params_starship_ext, input_params_sls_b1, input_params_sls_b2]
    optimize_configuration(; input_params...)
end

# df = CSV.File("/Users/mitchfogelson/Downloads/PET_global_params.csv", header=["name", "units", "value"]) |> DataFrame

# optimize_configuration(; input_params...)

# out = read_from_file("model_falcon.mof.json")


# keys = [:l1, :l2, :l3, :thickness, :α, :β, :n, :r, :phi, :h, :m]
# for key in keys
#     println(round(value.(model.obj_dict[key])[1], digits=3))
# end
# diameter = round(2*value(r) + 2*value(a[1]), digits=3)
# height = round(value(h[1])*value(m), digits=3)
# final_height = round(value(h[STATES])*value(m), digits=3)
# #######################################################################
# # Printing the results
# ## Super Structure
# println("The optimal height of the scissor is: ", value(m*h[1]))
# println("The optimal height of the scissor is: ", value(m*h[STATES]))

# ## Kresling
# println("The optimal radius of the kresling is: ", value(r))
# println("The optimal initial height of the kresling is: ", value(h[1]))
# println("The optimal final height of the kresling is: ", value(h[STATES]))
# println("The optimal number of cells is: ", value(m))
# println("The optimal initial angle of the kresling is: ", value(phi[1]))
# println("The optimal final angle of the kresling is: ", value(phi[STATES]))
# println("The optimal initial length of the long member is: ", value(lAC[1]))
# println("The optimal final length of the long member is: ", value(lAC[STATES]))
# println("The optimal initial length of the short member is: ", value(lBC[1]))
# println("The optimal initial length of the short member is: ", value(lBC[STATES]))

# println("The optimal length of the scissor is: ", round(value(l1), digits=3))
# println("The optimal length of the scissor is: ", round(value(l2), digits=3))
# println("The optimal length of the scissor is: ", round(value(l3), digits=3))
# println("The optimal thickness of the scissor is: ", round(value(thickness), digits=3))
# println("The optimal initial length of the scissor is: ", value(scissor_length[1]))
# println("The optimal final length of the scissor is: ", value(scissor_length[STATES]))
# println("The optimal number of scissors is: ", value(n))
# println("The optimal angle of the scissor is: ", round(value(α[1]), digits=3))
# println("The optimal angle of the scissor is: ", value(α[STATES]))
# println("The optimal angle of the scissor is: ", round(value(β[1]), digits=3))
# println("The optimal angle of the scissor is: ", value(β[STATES]))

# #######################################################################
# # Save decision variables 
# using JLD2
# config_params = model.obj_dict
# @save "falcon_model.jld2" config_params
# out = load("falcon_model.jld2")


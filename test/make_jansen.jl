using Dojo

function get_fixed_ids(mechanism::Mechanism)
    ids_leg1 = [body.id for body in mechanism.bodies if occursin("leg1", string(body.name))]
    ids_m = [body.id for body in mechanism.bodies if occursin("bar_m", string(body.name))]
    ids_a = [body.id for body in mechanism.bodies if occursin("bar_a", string(body.name))]
    ids_l = [body.id for body in mechanism.bodies if occursin("bar_l", string(body.name))]
    crank = [body.id for body in mechanism.bodies if occursin("crank", string(body.name))]
    crossbar = [body.id for body in mechanism.bodies if occursin("crossbar", string(body.name))]

    fixedids = [crank..., crossbar...]
    return fixedids
end

function make_jansen_small()
    path = joinpath(@__DIR__, "../DojoEnvironments/src/strandbeest/deps/Strandbeest.urdf")
    mechanism =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    for i in [6]
        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = Dojo.RotZ(pi)
    end
    return mechanism
end

function make_jansen_pair(pair_id::Int64)
    path = joinpath(@__DIR__, "../DojoEnvironments/src/strandbeest/deps/Strandbeest_0$pair_id.urdf")
    mechanism =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    for i in [pair_id]
        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_a_c")).rotational.orientation_offset*Dojo.RotZ(pi)
    end
    return mechanism
end

function make_jansen_full()
    path = joinpath(@__DIR__, "../DojoEnvironments/src/strandbeest/deps/Strandbeest_full.urdf")
    mechanism =  Mechanism(path; floating=true, gravity=[0., 0., -9.81], timestep=1e-3, parse_dampers=true)
    for i in 1:6
        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_f_g")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_f_g")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_b_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_b_c")).rotational.orientation_offset

        get_joint(mechanism, Symbol("pair0$(i)_leg2_loop_a_c")).rotational.orientation_offset = get_joint(mechanism, Symbol("pair0$(i)_leg1_loop_a_c")).rotational.orientation_offset*Dojo.RotZ(pi)
    end
    return mechanism
end
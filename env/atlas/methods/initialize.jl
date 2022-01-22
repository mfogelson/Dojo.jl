
function getatlas(; Δt::T = 0.01, g::T = -9.81, cf::T = 0.8, spring::T = 0.0,
        damper::T = 0.0, contact::Bool = true, model_type::Symbol = :simple) where {T}
    path = joinpath(@__DIR__, "../deps/atlas_$(string(model_type)).urdf")
    mech = Mechanism(path, true, T, g = g, Δt = Δt, spring=spring, damper=damper)

    # Adding springs and dampers
    for (i,eqc) in enumerate(collect(mech.eqconstraints)[2:end])
        eqc.isdamper = true
        eqc.isspring = true
        for joint in eqc.constraints
            joint.spring = spring
            joint.damper = damper
        end
    end

    if contact
        origin = Origin{T}()
        bodies = Vector{Body{T}}(collect(mech.bodies))
        eqs = Vector{EqualityConstraint{T}}(collect(mech.eqconstraints))

        # Foot contact
        contacts = [
            [-0.1; -0.05; 0.0-0.0095],
            [+0.1; -0.05; 0.0-0.0095],
            [-0.1; +0.05; 0.0-0.0095],
            [+0.1; +0.05; 0.0-0.0095],
            ]
        n = length(contacts)
        normal = [[0;0;1.0] for i = 1:n]
        offset = [[0.0; 0.0; 0.01] for i = 1:n]
        cf = cf * ones(T, n)
        names = ["RR", "FR", "RL", "RR"]

        ineqcs1 = contactconstraint(getbody(mech, "l_foot"), normal, cf=cf, p=contacts, offset=offset, names="l_" .* names)
        ineqcs2 = contactconstraint(getbody(mech, "r_foot"), normal, cf=cf, p=contacts, offset=offset, names="r_" .* names)

        setPosition!(mech, geteqconstraint(mech, "auto_generated_floating_joint"), [0;0;0.9385;0.;0.;0.])
        mech = Mechanism(origin, bodies, eqs, [ineqcs1; ineqcs2], g = g, Δt = Δt, spring=spring, damper=damper)
    end
    return mech
end

function initializeatlas!(mechanism::Mechanism;
    tran::AbstractVector{T} = [0,0,0.2],
    rot::AbstractVector{T} = [0,0,0.],
    v=[zeros(3) for i = 1:length(mechanism.bodies)],
    ω=[zeros(3) for i = 1:length(mechanism.bodies)],
    αhip::T = 0.0, αknee::T = 0.0) where {T}
    tran += [0,0,0.9385]

    # positions
    try
        setPosition!(mechanism,
                geteqconstraint(mechanism, "auto_generated_floating_joint"),
                [tran; rot])
        setPosition!(mechanism, geteqconstraint(mechanism, "l_leg_hpxyz"), [0.0, -αhip, 0.0])
        setPosition!(mechanism, geteqconstraint(mechanism, "r_leg_hpxyz"), [0.0, -αhip, 0.0])
        setPosition!(mechanism, geteqconstraint(mechanism, "l_leg_kny"), [αknee])
        setPosition!(mechanism, geteqconstraint(mechanism, "r_leg_kny"), [αknee])
        setPosition!(mechanism, geteqconstraint(mechanism, "l_leg_akxy"), [αhip-αknee, 0.0])
        setPosition!(mechanism, geteqconstraint(mechanism, "r_leg_akxy"), [αhip-αknee, 0.0])
    catch
        nothing
    end

    zeroVelocity!(mechanism)

    return nothing
end

function initializeatlasstance!(mechanism::Mechanism;
    tran::AbstractVector{T} = [0,0,0.2],
    rot::AbstractVector{T} = [0,0,0.],
    v=[zeros(3) for i = 1:length(mechanism.bodies)],
    ω=[zeros(3) for i = 1:length(mechanism.bodies)],
    αhip::T = 0.0, αknee::T = 0.0) where {T}
    tran += [0,0,0.9385]

    # positions
    try
        setPosition!(mech,
        geteqconstraint(mech, "auto_generated_floating_joint"),
        [[0,0,0.5]; [0.0,0.0, 0.0]])
        # setPosition!(mech, geteqconstraint(mech, "l_leg_hpxyz"), [0.0, -αhip, 0.0])
        # setPosition!(mech, geteqconstraint(mech, "r_leg_hpxyz"), [0.0, -αhip, 0.0])
        setPosition!(mech, geteqconstraint(mech, "l_leg_kny"), [αknee])
        setPosition!(mech, geteqconstraint(mech, "r_leg_kny"), [αknee])
        # setPosition!(mech, geteqconstraint(mech, "l_leg_akxy"), [αhip-αknee, 0.0])
        # setPosition!(mech, geteqconstraint(mech, "r_leg_akxy"), [αhip-αknee, 0.0])

        setPosition!(mech, geteqconstraint(mech, "auto_generated_floating_joint"), [tran; rot])
        setPosition!(mech, geteqconstraint(mech, "back_bkx"), [0.0  * π])
        setPosition!(mech, geteqconstraint(mech, "back_bky"), [0.04 * π])
        setPosition!(mech, geteqconstraint(mech, "back_bkz"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_elx"), [0.25 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_ely"), [0.5 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_shx"), [-0.5 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_shz"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_mwx"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "l_arm_uwy"), [0.0 * π])
        # setPosition!(mech, geteqconstraint(mech, "l_arm_lwy"), [0.0])
        # setPosition!(mech, geteqconstraint(mech, "l_leg_akx"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "l_leg_aky"), [-0.1 * π])
        # setPosition!(mech, geteqconstraint(mech, "l_leg_hpx"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "l_leg_hpy"), [-0.1 * π])
        # setPosition!(mech, geteqconstraint(mech, "l_leg_hpz"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "l_leg_kny"), [0.2 * π])
        setPosition!(mech, geteqconstraint(mech, "neck_ay"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "r_arm_elx"), [-0.25 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_ely"), [0.5 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_shx"), [0.5 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_shz"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_mwx"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_uwy"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "r_arm_lwy"), [0.0 * π])
        # setPosition!(mech, geteqconstraint(mech, "r_leg_akx"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "r_leg_aky"), [-0.1 * π])
        # setPosition!(mech, geteqconstraint(mech, "r_leg_hpx"), [0.0])
        setPosition!(mech, geteqconstraint(mech, "r_leg_hpy"), [-0.1 * π])
        # setPosition!(mech, geteqconstraint(mech, "r_leg_hpz"), [0.0 * π])
        setPosition!(mech, geteqconstraint(mech, "r_leg_kny"), [0.2 * π])
    catch
        nothing
    end

    zeroVelocity!(mechanism)

    return nothing
end

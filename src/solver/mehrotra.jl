"""
    mehrotra!(mechanism; opts)

    interior-point solver for simulation-step feasibility problem

    mechanism: Mechanism
    opts: SolverOptions
"""
function mehrotra!(mechanism::Mechanism{T}; opts=SolverOptions{T}()) where T
	reset!.(mechanism.contacts, scale=1.0) # resets the values of s and γ to the scaled neutral vector; TODO: solver option
	reset!.(mechanism.joints,   scale=1.0) # resets the values of s and γ to the scaled neutral vector; TODO: solver option

	status = :failed
    mechanism.μ = 0.0
	μtarget = 0.0
	no_progress = 0
    undercut = opts.undercut
    α = 1.0

	initialize!.(mechanism.contacts)
    set_entries!(mechanism, reg=opts.reg) # compute the residual

    bvio = bilinear_violation(mechanism) # does not require to apply set_entries!
    rvio = residual_violation(mechanism) # does not require to apply set_entries!
    
	opts.verbose && solver_header()
    for n = Base.OneTo(opts.max_iter)
        opts.verbose && solver_status(mechanism, α, rvio, bvio, n, μtarget, undercut)

        (rvio < opts.rtol) && (bvio < opts.btol) && (status=:success; break)
		(n == opts.max_iter) && (opts.verbose && (@warn "failed mehrotra"))

        # affine search direction
		μ = 0.0
		pull_residual!(mechanism)               # store the residual inside mechanism.residual_entries
        #!# Jan LDU Version 
        # ldu_factorization!(mechanism.system)    # factorize system, modifies the matrix in place
        # ldu_backsubstitution!(mechanism.system) # solve system, modifies the vector in place

        A = full_matrix(mechanism.system)
        b = full_vector(mechanism.system)
        #!# Full Matrix Version
        # out = A \ b

        #!# SVD Version 
        F = svd(A, full=true, alg=LinearAlgebra.QRIteration())
        rank = sum(F.S .> 1e-6)
        V1 = @view F.V[:,1:rank]
        S1 = @view F.S[1:rank]
        U1 = @view F.U[:,1:rank]

        out = V1*Diagonal(1.0 ./ S1)*U1'*b

        start = 0
        for i in eachindex(mechanism.system.vector_entries)
            mechanism.system.vector_entries[i].value = out[start + 1: start+size(mechanism.system.vector_entries[i].value, 1)]
            start += size(mechanism.system.vector_entries[i].value, 1)
        end

		αaff = cone_line_search!(mechanism; τort=0.95, τsoc=0.95) # uses system.vector_entries which holds the search drection
		ν, νaff = centering!(mechanism, αaff)
		σcentering = clamp(νaff / (ν + 1e-20), 0.0, 1.0)^3

		# corrected search direction
		μtarget = max(σcentering * ν, opts.btol / undercut)
		mechanism.μ = μtarget
		correction!(mechanism) # update the residual in mechanism.residual_entries

		push_residual!(mechanism)               # cache residual + correction
        b = full_vector(mechanism.system)
        #!# Full Matrix Version
        # out = A \ b

        #!# SVD Version 
        out = V1*Diagonal(1.0 ./ S1)*U1'*b

        start = 0
        for i in eachindex(mechanism.system.vector_entries)
            mechanism.system.vector_entries[i].value = out[start+1: start + size(mechanism.system.vector_entries[i].value, 1)]
            start += size(mechanism.system.vector_entries[i].value, 1) 
        end
        # mechanism.system.vector_entries .= A \ b
        # ldu_backsubstitution!(mechanism.system) # solve system

		τ = max(0.95, 1 - max(rvio, bvio)^2) # τ = 0.95
		α = cone_line_search!(mechanism; τort=τ, τsoc=min(τ, 0.95)) # uses system.vector_entries which holds the corrected search drection

		# steps taken without making progress
		rvio_, bvio_ = line_search!(mechanism, α, rvio, bvio, opts)

        # evaluate progress
		made_progress = (!(rvio_ < opts.rtol) && (rvio_ < 0.8rvio)) || (!(bvio_ < opts.btol) && (bvio_ < 0.8bvio)) # we only care when progress is made while the tolerance is not met
		made_progress ? no_progress = max(no_progress - 1, 0) : no_progress += 1
		rvio, bvio = rvio_, bvio_
		(no_progress >= opts.no_progress_max) && (undercut *= opts.no_progress_undercut)

		# update solution
        update!.(mechanism.bodies)
        update!.(mechanism.joints)
        update!.(mechanism.contacts)

		# recompute Jacobian and residual
        set_entries!(mechanism, reg=opts.reg)
    end

    return status
end

function solver_status(mechanism::Mechanism, α, rvio, bvio, n, μtarget, undercut)
    fv = full_vector(mechanism.system)
    Δvar = norm(fv, Inf)
    # fM = full_matrix(mechanism.system)
    # fΔ = fM \ fv
    # Δalt = norm(fΔ, Inf)
    res = norm(fv, Inf)
	println(
        n,
        "   ", scn(bvio, digits=0),
        "   ", scn(rvio, digits=0),
        "   ", scn(α, digits=0),
        "   ", scn(μtarget, digits=0),
        "   ", scn(res, digits=0),
        "   ", scn(Δvar, digits=0),
        # "   ucut", scn(undercut),
        )
end

function solver_header()
	println("                                                 ")
	println("n    bvio    rvio     α       μ     |res|∞   |Δ|∞")
	println("–––––––––––––––––––––––––––––––––––––––––––––––––")
end


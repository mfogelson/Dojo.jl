joint_types = [
    :Fixed,
    :Prismatic,
    :Planar,
    :FixedOrientation,
    :Revolute,
    :Cylindrical,
    :PlanarAxis,
    :FreeRevolute,
    :Orbital,
    :PrismaticOrbital,
    :PlanarOrbital,
    :FreeOrbital,
    :Spherical,
    :CylindricalFree,
    :PlanarFree
    ]

function test_get_set_data(mechanism::Mechanism)
    Nd = Dojo.data_dim(mechanism,
		attjac=false)
    data0 = rand(Nd)
    Dojo.set_data!(mechanism, data0)
    data1 = Dojo.get_data(mechanism)
    @test norm(data0 - data1) < 1.0e-8
end

@testset "Get and set data" begin
    mech = DojoEnvironments.get_snake(;
		num_bodies=3, dampers=1.0, springs=1.0, contact_type=:nonlinear);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_snake(;
		num_bodies=3, dampers=1.0, springs=1.0, contact_type=:linear);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_snake(;
		num_bodies=3, dampers=1.0, springs=1.0, contact_type=:impact);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_pendulum(;
		dampers=1.0, springs=10.0);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_humanoid(;
		parse_springs=false, parse_dampers=false,
		springs=10.0, contact_feet=true);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_humanoid(;
		parse_springs=false, parse_dampers=false,
		springs=10.0, contact_feet=false);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_atlas(;
		parse_springs=false, parse_dampers=false,
		springs=10.0, dampers=10.0);
    test_get_set_data(mech)

    mech = DojoEnvironments.get_quadruped(;
		parse_springs=false, parse_dampers=false,
		dampers=1.0, springs=10.0);
    test_get_set_data(mech)
end


################################################################################
# Analytical Jacobian
################################################################################
# Controller
function ctrl!(mechanism, k)
	nu = Dojo.input_dimension(mechanism)
	if Dojo.input_dimension(mechanism.joints[1]) == 6
		u = 0.2 * [szeros(6); sones(nu-6)]
	else
		u = 0.2 * sones(nu)
	end
	Dojo.set_input!(mechanism, u)
	return
end

function test_data_system(model::Symbol;
		ϵ=1.0e-6,
		tsim=0.1,
		ctrl=(m, k)->nothing,
        timestep=0.01,
		gravity=[0.0; 0.0; -9.81],
		verbose=false,
		T=Float64,
		kwargs...)

    # mechanism
      mechanism = DojoEnvironments.get_mechanism(model;
		timestep,
		gravity,
		kwargs...)
    Dojo.initialize!(mechanism, model)
    # simulate
    Dojo.simulate!(mechanism, tsim, ctrl!,
        record=false,
		verbose=false,
		opts=Dojo.SolverOptions(rtol=ϵ, btol=ϵ))

	# Finite Difference
	Nd = Dojo.data_dim(mechanism,
		attjac=false)
	data0 = Dojo.get_data(mechanism)
	sol0 = Dojo.get_solution(mechanism)
	datajac0 = Dojo.finite_difference_data_jacobian(mechanism, data0, sol0)
	attjac0 = Dojo.data_attitude_jacobian(mechanism)
	datajac0 *= attjac0

	# Analytical
	D = Dojo.create_data_matrix(mechanism.joints, mechanism.bodies, mechanism.contacts)
	Dojo.jacobian_data!(D, mechanism)
	nodes = [mechanism.joints; mechanism.bodies; mechanism.contacts]
	dimrow = length.(nodes)
	dimcol = Dojo.data_dim.(nodes)
	datajac1 = Dojo.full_matrix(D, false, dimrow, dimcol)

	# Test
	@testset "$(String(model))" begin
		@test norm(datajac0 - datajac1, Inf) < 1.0e-6
	end
    return nothing
end

################################################################################
# Without contact and joint limits
################################################################################
@testset "Data Jacobian without contact and limits" begin
	for (springs, dampers) in [(0.0, 0.0), (2.0, 0.3)]
		test_data_system(:sphere;
			contact=false)
		test_data_system(:block;
			contact=false)
		test_data_system(:block2d;
			contact=false)
		test_data_system(:slider;
			springs, dampers)
		test_data_system(:nslider;
			springs, dampers)
		test_data_system(:pendulum;
			springs, dampers)
		test_data_system(:cartpole;
			springs, dampers)
		test_data_system(:pendulum;
			springs, dampers)
		test_data_system(:hopper;
			parse_springs=false,
			parse_dampers=false,
			springs, dampers,
			contact_foot=false,
			contact_body=false)
		test_data_system(:humanoid;
			parse_springs=false,
			parse_dampers=false,
			springs, dampers,
			contact_feet=false)
		test_data_system(:atlas;
			parse_springs=false,
			parse_dampers=false,
			springs, dampers,
			contact_feet=false,
			contact_body=false)
		test_data_system(:halfcheetah;
			parse_springs=false,
			parse_dampers=false,
			contact_feet=false,
			contact_body=false,
			joint_limits=Dict())
		test_data_system(:walker; 
			parse_springs=false,
			parse_dampers=false,
			springs, dampers,
			contact_feet=false,
			contact_body=false,
			joint_limits=Dict())
		test_data_system(:quadruped; 
			parse_springs=false,
			parse_dampers=false,
			springs, dampers, 
			contact_feet=false,
			contact_body=false,
			joint_limits=Dict())
		for joint_type in joint_types
			test_data_system(:snake;
				num_bodies=5,
				springs, dampers,
				contact=false,
				joint_type)
			test_data_system(:twister;
				num_bodies=5,
				springs, dampers,
				contact=false,
				joint_type)
		end
	end
end

################################################################################
# With contact and joint limits
################################################################################
@testset "Data Jacobian with contact and limtis" begin
	for (springs, dampers) in [(0.0, 0.0), (2.0, 0.3)]
		test_data_system(:sphere;
			contact=true)
		test_data_system(:block;
			contact=true)
		test_data_system(:block2d;
			contact=true)
		test_data_system(:slider;
			springs,
			dampers)
		test_data_system(:nslider;
			springs,
			dampers)
		test_data_system(:pendulum;
			springs,
			dampers)
		test_data_system(:cartpole;
			springs,
			dampers)
		test_data_system(:pendulum;
			springs,
			dampers)
		test_data_system(:hopper;
			parse_springs=false,
			parse_dampers=false,
			springs,
			dampers,
			contact_foot=true,
			contact_body=true)
		test_data_system(:humanoid;
			parse_springs=false,
			parse_dampers=false,
			springs,
			dampers,
			contact_feet=true)
		test_data_system(:atlas;
			parse_springs=false,
			parse_dampers=false,
			springs,
			dampers,
			contact_feet=true,
			contact_body=true)
		test_data_system(:halfcheetah;
			parse_springs=false,
			parse_dampers=false,
			contact_feet=true,
			contact_body=true)
		test_data_system(:walker;
			parse_springs=false,
			parse_dampers=false,
			springs,
			dampers,
			contact_feet=true,
			contact_body=true)
		test_data_system(:quadruped;
			parse_springs=false,
			parse_dampers=false,
			springs,
			dampers,
			contact_feet=true,
			contact_body=true)
		for joint_type in joint_types
			test_data_system(:snake;
				num_bodies=5,
				springs,
				dampers,
				contact=true,
				joint_type=joint_type)
			test_data_system(:twister;
				num_bodies=5,
				springs,
				dampers,
				contact=true,
				joint_type=joint_type)
		end
	end
end

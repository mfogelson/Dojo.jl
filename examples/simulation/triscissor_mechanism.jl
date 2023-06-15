# ### Setup
# PKG_SETUP
using Dojo
using DojoEnvironments

# ### Get mechanism (check DojoEnvironment/mechanisms files for kwargs)
mechanism = get_mechanism(:triscissor, contact=false, gravity=0.0, radius=0.05, len=1.0, scale=1.0, num_sets=1) 

# build_robot

# ### Initialize mechanism (check DojoEnvironment/mechanisms files for kwargs)[]
# initialize!(mechanism, :triscissor)

# ### Simulate mechanism
storage = simulate!(mechanism, 0.01, record=true)
    
# ### Visualize mechanism
vis = visualize(mechanism, storage, show_joint=true, show_frame=true)
render(vis)
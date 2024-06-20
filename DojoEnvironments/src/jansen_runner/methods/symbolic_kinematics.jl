using LinearAlgebra
xi = [[-0.2       , -0.85460342, -1.4836573 , -1.79282032,
-1.63743169, -1.09019946, -0.40717968,  0.09203511,
 0.17385675, -0.2       ];; 
[ 1.        ,  1.19850812,  0.92980319,  0.31961524,
-0.34654201, -0.75696618, -0.71961524, -0.25196611,
 0.42716299,  1.        ]]

xi = reshape(xi,10, 2)'

xj = [[ 0.6       ,  0.6       ,  0.6       ,  0.6       ,
                 0.6       ,  0.6       ,  0.6       ,  0.6       ,
                 0.6       ,  0.6       ];
               [-0.2       , -0.2       , -0.2       , -0.2       ,
                -0.2       , -0.2       , -0.2       , -0.2       ,
                -0.2       , -0.2       ]]
xj = reshape(xj,10, 2)'
xk0 = [-1.0, -0.6]


function symbolic_kinematics(xi::AbstractArray, xj::AbstractArray, xk0::AbstractArray)
    """Symbolic Kinematics implementation
    Args:
        xi (Nd.Array): Path of revolute joint xi Shape: (2, n)
        xj (Nd.Array): Path of revolute joint xj Shape: (2, n)
        xk0 (Nd.Array): Initial position of new point xk Shape: (2,)
    Returns:
        Nd.Array: Path of revolute joint xk Shape: (2,n)
    """

    _, N = size(xi)

    l_ij = map(norm, eachslice((xj-xi), dims=2));
    l_ik = norm(xi[:,1] - xk0); # float
    l_jk = norm(xj[:,1] - xk0); # float

    # Check Triangle Inequality (Quick Heuristic to see if link will return valid solution)
    valid = all(l_ik+l_jk .> l_ij) & 
            all(l_ik.+l_ij .> l_jk) &
            all(l_jk.+l_ij .> l_ik) &
            all(l_ij .> 0) &
            all(l_ik .> 0)

    # If not Valid Return all NaN
    if !valid
        return fill!(similar(xi), NaN)
    end

    f = l_ik ./ l_ij # (N, )

    t = @. (l_ij^2 + (l_ik^2 - l_jk^2))/(2*l_ij*l_ik) # (N, )
    R1 = @. [t -sqrt(1.0 - t^2)]*f
    R2 = @. [sqrt(1.0 - t^2) t]*f # (2, 2, N)
    # Q = (R.*f) # (N, 2, 2)

    diff = xj-xi # ()
    # diff = reshape(diff, (1,size(diff)...)) #diff[np.newaxis, :,:].T
    xk = zero(similar(xi))
    for i in 1:N
        xk[:, i] = ([R1[i, :] R2[i,:]]*reshape(diff[:,i], 2, 1))
    end 
    xk += xi

    ## found solution path
    if norm(xk[:,1]-xk0) < 1e-3
        return xk
    end

    ## flip orientation
    R1 = @. [t sqrt(1.0 - t^2)]*f
    R2 = @. [-sqrt(1.0 - t^2) t]*f # (2, 2, N)
    
    xk = zero(similar(xi))
    for i in 1:N
        xk[:, i] = ([R1[i, :] R2[i,:]]*reshape(diff[:,i], 2, 1))'
    end 
    xk += xi

    ## found solution path
    if norm(xk[:,1]-xk0) < 1e-3
        return xk
    end

    ## Passes through singularity / invalid
    return fill!(similar(xi), NaN)
end
xk_ = reshape([[-1.        , -1.0038898 , -0.94533894, -0.75629343,
-0.32690443,  0.3419011 ,  1.02305462,  0.98001473,
-0.79833228, -1.        ];
[-0.6       , -0.58410613, -0.77613155, -1.13833263,
-1.56412909, -1.82892141, -1.79405922, -1.80486411,
-1.0744523 , -0.6       ]], 10, 2)'

xk = symbolic_kinematics(xi, xj, xk0)
@assert norm(xk-xk_) < 1e-6

# function initialize_closed_chian()
#     """Update self.paths
#     Args:
#         unknown_joints (list, optional): node indexes that are not known or want to be calculated. Defaults to None.
#     """
#     n = self.number_of_nodes()
    
#     if unknown_joints is None:
#         known_joints = list(np.argwhere(self.node_type == 0)[:,0])
#         known_joints.append(1) #(np.argwhere(self.adj[0,:] == 1).item()) #TODO: Fix this
#         unknown_joints = list(set(range(n)) ^ set(known_joints)) 
#     else:
#         assert isinstance(unknown_joints, list)
#         known_joints = list(set(range(n)) ^ set(unknown_joints))


#     count = 0
#     while list(set(range(n)) ^ set(known_joints)) != [] and count < 100:

#         for i in unknown_joints[:]:
            
#             if sum(self.adj[i, known_joints]) >= 2:


#                 inds = np.array(known_joints)[np.where(self.adj[i, known_joints] >= 1)[0]]

#                 # Update paths
#                 self.paths[i, :, :] = symbolic_kinematics(self.paths[inds[0],:,:], self.paths[inds[1], :, :], self.paths[i, :, 0])
                
#                 unknown_joints.remove(i)
#                 known_joints.append(i)
#             else:
#                 pass
#         count += 1
# end
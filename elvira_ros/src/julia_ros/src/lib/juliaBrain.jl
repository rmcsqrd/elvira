using RobotOS
using Serialization

# function that unpacks blob and places into an array
function juliaBrain(blobArray, experiment_type, Q1_mat, Q2_mat, actions, γ, α, ϵ, A, S, n_bins, dimsize)

    # figure out state from previous action and observe reward. Also choose action
    Sp_unmod = stateGen(blobArray, n_bins, dimsize) # returns something like [0.0, 0.1, ...]
    println("state space: $Sp_unmod")
    Sp = binAssign(Sp_unmod, dimsize)
    #1 choose action
    Ap = actionGen(Sp, Q1_mat, actions, ϵ)
    action_string = actions[Ap]
    
    # determine reward
    S = deserialize(string(@__DIR__, "/S"))
    A = deserialize(string(@__DIR__, "/A"))
    reward = rewardGen(Sp_unmod, A, dimsize)
    
    # display stuff
    println("reward: $reward")
    println("action command: $action_string")
    println("\n")
    
    # select Q matrix
    plotinfo = deserialize(string(@__DIR__, "/plotinfo"))
    Q1_mat = updateQ(Q1_mat, Sp, Ap, γ, α, actions, reward, S, A)
    
    # reassign A, S variables
    serialize(string(@__DIR__, "/S"), Sp[1:n_bins])
    serialize(string(@__DIR__, "/A"), actions[Ap])
    print(size(Q1_mat))
    push!(plotinfo, [reward, Ap, sum(Q1_mat[:,:,:,1]), sum(Q1_mat[:,:,:,2]), sum(Q1_mat[:,:,:,3])])
    serialize(string(@__DIR__, "/plotinfo"), plotinfo) 

    # take action
    return action_string 
end

function Qinit(A, n_bins, dimsize)
    # setup S, A, T, γ, R tuple
    # blobArray is Nx5, where N = number of blobs.
    # [icm, jcm, N, rad, status]
   
    
    # this is ugly but takes n_bins and appends action space
    matdims = Tuple(if x == n_bins+1 size(A)[1] else dimsize end for x in 1:n_bins+1)  
    
    # generate Q matrices 
    Q1_mat = zeros(Float64, (matdims))
    return Q1_mat
end

function stateGen(blobArray, n_bins, dimsize)
    
    # define geometries
    blobArray = transpose(blobArray)
    icm_list = blobArray[1, :]
    jcm_list = blobArray[2, :]
    N_list = blobArray[3, :]
    rad_list = blobArray[4, :]
    status_list = blobArray[5, :]

    h = 480
    w = 640

    resultDist = zeros(n_bins)

    for k in 1:size(blobArray)[2]
        
        # represent circle as rectangle, preserve width (wp) and adjust height (hp) for 1d bins
        wp = 2*rad_list[k]
        hp = (3.1415*rad_list[k]^2)/wp

        # loop through bins to comupte occupancy of portion of rectangle in each bin
        bin_i = 0
        bin_width = floor(w/n_bins)
        wmin = icm_list[k]-wp/2
        wmax = icm_list[k]+wp/2

        for i in 1:size(resultDist)[1]
            bin_i += bin_width                
            if bin_i > wmin && (bin_i - bin_width) < wmin
                resultDist[i] += (bin_i-wmin)*hp/(h*bin_width)
            elseif bin_i > wmin && (bin_i - bin_width) > wmin && (bin_i-bin_width) < wmax
                resultDist[i] += bin_width*hp/(h*bin_width)
            elseif bin_i > wmax && (bin_i - bin_width) < wmax
                resultDist[i] += (wmax-(bin_i-bin_width))*hp/(h*bin_width) 
            end
        end        
    end

    resultDist = round.(resultDist, digits=1)
    return resultDist
end

function rewardGen(S, action, dimsize)
    statesize = size(S)[1]
    lowerthird = convert(Int, floor(statesize*0.33))
    upperthird = convert(Int, ceil(statesize*0.66))
    reward = 0
    for i in lowerthird+1:upperthird
        if S[i] > 1/dimsize
            reward -= 100
        end
    end
    if action != "noRotate" && reward < 0
        reward -= 20
    elseif action != "noRotate"
        reward -= 10
   
    elseif reward == 0 && action == "noRotate"
        reward += 20
    end
    
    return reward
end

function actionGen(Sp,  Q1_mat, actions, ϵ)
    Sp = CartesianIndex(Tuple(vec(Sp)))
    
    # use ϵ-greedy to determine Q
    if rand(1:100) > ϵ*100
        if sum(Q1_mat[Sp, :]) == 0.0  # check if array empty, if yes do random
            Ap = rand(1:size(actions)[1])
        else
            Ap = findmax(Q1_mat[Sp, :])[2]  # returns (val, index)
        end
    else
        Ap = rand(1:size(actions)[1])
    end
end

function updateQ(Q1_mat, Sp, Ap, γ, α, actions, reward, S, A)
    Ap_index = Ap 
    A_index = findall(x->x==A, actions)[1]
    QSpAp = CartesianIndex(Tuple(push!(vec(Sp), Ap_index)))
    QSA = CartesianIndex(Tuple(push!(vec(S), A_index)))
    println("QSpAp = $QSpAp")
    println("QSA = $QSA")
    Q1_mat[QSA]  += α*(reward+γ*Q1_mat[QSpAp]-Q1_mat[QSA])
    loginfo(Q1_mat)
    return Q1_mat
    
end

function binAssign(array, dimsize)
# just a quick and dirty helper function to group bins
           result = []
           for item in array
               inc = 1/dimsize
               i = 0
               while item >= i*inc && item <= 1
                   if item == 1
                       i = inc^-1
                       break
                   elseif item == 0.0
                       i = 1
                       break
                   else
                       i+=1
                   end
               end
           push!(result, i)
           end
       return convert.(Int, result)
       end


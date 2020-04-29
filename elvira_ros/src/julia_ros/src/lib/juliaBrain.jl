using RobotOS

# function that unpacks blob and places into an array
function juliaBrain(blobArray, experiment_type, Q1_mat, Q_2_mat)

    loginfo(size(Q1_mat))
    return "dummy_string"
end

function Qinit()
    # setup S, A, T, γ, R tuple
    # blobArray is Nx5, where N = number of blobs.
    # [icm, jcm, N, rad, status]
    n_bins = 2 # vertical image partition bins
    dimsize = 10
   
    A = ["CW", "CCW", "noRotate"]
    T = 1.0
    γ = 0.99
    
    # this is ugly but takes n_bins and appends action space
    matdims = Tuple(if x == n_bins+1 size(A)[1] else dimsize end for x in 1:n_bins+1)  

    # generate Q matrices 
    Q1_mat = zeros(Float64, (matdims))
    return Q1_mat
end

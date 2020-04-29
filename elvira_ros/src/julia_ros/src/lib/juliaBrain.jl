using RobotOS

# function that unpacks blob and places into an array
function juliaBrain(blobArray, experiment_type, Q1_mat, Q2_mat)
    loginfo(blobArray)
    
    # process geometry data for bin percentages for current state
    n_bins = size(Q1_mat)[1]-1
    S = binDistributions(blobArray, n_bins)
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

function binDistributions(blobArray, n_bins)
    
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
            elseif bin_i > wmin && (bin_i - bin_width) > wmin
                resultDist[i] += bin_width*hp/(h*bin_width)
            elseif bin_i > wmax && (bin_i - bin_width) < wmax
                resultDist[i] += (wmax-(bin_i-bin_width))*hp/(h*bin_width) 
            end
        end        
    end

    resultDist = round.(resultDist, digits=1)
    println("\n $resultDist \n")

    return resultDist
end






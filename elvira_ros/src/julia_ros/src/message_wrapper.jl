#!/home/elvira/programming/apps/julia/bin/julia

# High level framework stuff/imports
ENV["PYTHON"]="/usr/bin/python2.7"
using RobotOS
using Serialization
@rosimport geometry_msgs.msg: Point, Pose2D
@rosimport std_msgs.msg.String # note this imports as type StringMsg due to conflict
@rosimport std_msgs.msg.Int32MultiArray
rostypegen()

include(string(@__DIR__, "/lib/juliaBrain.jl"))

# include msg stuff
import .std_msgs.msg.StringMsg
import .std_msgs.msg.Int32MultiArray

# multi_blob subscribe/publish
function callback(msg::Int32MultiArray, motor_pub_obj::Publisher{StringMsg}, visual_pub_obj::Publisher{Int32MultiArray}, Q1_mat, Q2_mat, actions, γ, α, ϵ, sleep_rate, n_bins, dimsize)

    # unpack and convert data to an array
    blobVect = msg.data  
    num_blobs = convert(Int, size(blobVect)[1]/5)
    
    if num_blobs == 0  # handle case where no blobs by passing dummy data
        blobVect = [0, 0, 0, 0, 0] 
        num_blobs = 1
    end
        A = deserialize(string(@__DIR__, "/lib/A"))
        S = deserialize(string(@__DIR__, "/lib/S"))
        blobArray = transpose(reshape(blobVect, :, num_blobs))
        raw_string = juliaBrain(blobArray, "scaredy_cat", Q1_mat, Q2_mat, actions, γ, α, ϵ, A, S, n_bins, dimsize)
    
    text = StringMsg(raw_string)
    publish(motor_pub_obj, text)
    publish(visual_pub_obj, msg)
    #rossleep(sleep_rate)

end


function main()
    try
        rm(string(@__DIR__, "/lib/S"))
        rm(string(@__DIR__, "/lib/A"))
    catch
    end
    init_node("julia_node")

    actions = ["CW", "CCW", "noRotate"]
    n_bins = 3 # vertical image partition bins
    dimsize = 3 # [0.0, 0.1, 0.2, ...,0.9, 1.0]
    
    A = "noRotate"
    # initalize state space stuff
    Q1_mat = Qinit(actions, n_bins, dimsize)
    Q2_mat = deepcopy(Q1_mat)
    S = ones(length(size(Q1_mat))-1)
    S = convert.(Int, S)
    serialize(string(@__DIR__, "/lib/S"), S)
    serialize(string(@__DIR__, "/lib/A"), A)

    γ = 0.99
    α = 0.5
    ϵ = 0.1
    sleep_rate = 0.0
    
    motor_pub = Publisher{StringMsg}("/julia_brain/motor_control", queue_size=1);
    visual_pub = Publisher{Int32MultiArray}("/julia_brain/visual_out", queue_size=1);
    #sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (motor_pub, visual_pub,Q1_mat, Q2_mat, actions, γ, α, ϵ, sleep_rate, ), queue_size=1)
    sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (motor_pub, visual_pub,Q1_mat, Q2_mat, actions, γ, α, ϵ, sleep_rate, n_bins, dimsize, ))
    while ! is_shutdown()
        spin()
        rossleep(sleep_rate)
    end
        
    
end

if ! isinteractive()
    main()
end

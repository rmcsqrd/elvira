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
function callback(msg::Int32MultiArray, motor_pub_obj::Publisher{StringMsg}, visual_pub_obj::Publisher{Int32MultiArray}, Q1_mat, Q2_mat)

    # unpack and convert data to an array
    blobVect = msg.data  
    num_blobs = convert(Int, size(blobVect)[1]/5)
    blobArray = transpose(reshape(blobVect, :, num_blobs))
    
    
    raw_string = juliaBrain(blobArray, "scaredy_cat", Q1_mat, Q2_mat)
    text = StringMsg(raw_string)
    publish(motor_pub_obj, text)
    publish(visual_pub_obj, msg)

end

# node loop function 
function loop()
    loop_rate = Rate(1.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end
end

function main()
    cacheclr = 1
    serialize("cacheclr_flag", cacheclr)  # this forces overwrite of stored Q matrix between runs`

    init_node("julia_node")

    # initalize state space stuff
    Q1_mat = Qinit()
    Q2_mat = deepcopy(Q1_mat)

    motor_pub = Publisher{StringMsg}("/julia_brain/motor_control", queue_size=10);
    visual_pub = Publisher{Int32MultiArray}("/julia_brain/visual_out", queue_size=10);
    sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (motor_pub, visual_pub,Q1_mat, Q2_mat,  ), queue_size=10)
    loop()
end

if ! isinteractive()
    main()
end

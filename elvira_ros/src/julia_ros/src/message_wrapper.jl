#!/home/elvira/programming/apps/julia/bin/julia

# High level framework stuff/imports
ENV["PYTHON"]="/usr/bin/python2.7"
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
@rosimport std_msgs.msg.String # note this imports as type StringMsg due to conflict
@rosimport std_msgs.msg.Int32MultiArray
rostypegen()

include(string(@__DIR__, "/lib/juliaBrain.jl"))

# include msg stuff
import .std_msgs.msg.StringMsg
import .std_msgs.msg.Int32MultiArray

# multi_blob subscribe/publish
function callback(msg::Int32MultiArray, pub_obj::Publisher{StringMsg})

    # unpack and convert data to an array
    blobVect = msg.data  
    num_blobs = convert(Int, size(blobVect)[1]/5)
    blobArray = transpose(reshape(blobVect, :, num_blobs))
    logwarn(blobArray)
    #raw_string = 
    text = StringMsg("pushup")
    publish(pub_obj, text)
end

# node loop function 
function loop()
    loop_rate = Rate(1.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end
end

function main()
    init_node("julia_node")
    pub = Publisher{StringMsg}("motor_control", queue_size=10);
    sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (pub,), queue_size=10)
    loop()
end

if ! isinteractive()
    main()
end

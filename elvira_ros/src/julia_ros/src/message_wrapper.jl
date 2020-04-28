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
function callback(msg::Int32MultiArray, motor_pub_obj::Publisher{StringMsg})#, visual_pub_obj::Publisher{Int32MultiArray})

    # unpack and convert data to an array
    blobVect = msg.data  
    num_blobs = convert(Int, size(blobVect)[1]/5)
    blobArray = transpose(reshape(blobVect, :, num_blobs))
    
    # call the juliaBrain.jl wrapper function and return a command in the form of a string
    
    text = StringMsg("raw_string")
    publish(motor_pub_obj, text)
    #publish(visual_pub_obj, msg.data)

    rosinfo("got here")
end

# node loop function 
function loop()
    loop_rate = Rate(1.0)
    while ! is_shutdown()
        rossleep(loop_rate)
    end
end

function main()
    init_node("julia_brain")
    motor_pub = Publisher{StringMsg}("/julia_brain/motor_control", queue_size=10)
    #visual_pub = Publisher{Int32MultiArray}("/julia_brain/visual_output", queue_size=10)
    #sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (motor_pub, visual_pub, ), queue_size=10)
    sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, (motor_pub, ), queue_size=10)
    loop()
end

if ! isinteractive()
    main()
end

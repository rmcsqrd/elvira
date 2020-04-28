#!/home/elvira/programming/apps/julia/bin/julia


# High level framework stuff/imports
ENV["PYTHON"]="/usr/bin/python2.7"
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
@rosimport std_msgs.msg.String # note this imports as type StringMsg due to conflict
@rosimport std_msgs.msg.Int32MultiArray
rostypegen()

# include msg stuff
import .std_msgs.msg.StringMsg
import .std_msgs.msg.Int32MultiArray

# multi_blob subscribe
function callback(msg::Int32MultiArray)

# raw message looks like
# Int32MultiArray(Main.std_msgs.msg.MultiArrayLayout(Main.std_msgs.msg.MultiArrayDimension[], 0x00000000), Int32[536, 253, 4672, 38, 1, 80, 296, 4297, 36, 1])
    #blob_data = msg[2]
    loginfo("latency check")
    #loginfo(blob_data)
end

# motor_driver publish
function loop(pub_obj)
    loop_rate = Rate(1.0)
    while ! is_shutdown()
        text = StringMsg("pushup")
        publish(pub_obj, text)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_example")
    pub = Publisher{StringMsg}("motor_control", queue_size=10);
    sub = Subscriber{Int32MultiArray}("/multi_blob/blob_data", callback, queue_size=10)
    
    #cb1(msg::Imu, a::String) = println(a,": ",msg.linear_acceleration.x)

    loop(pub)
end

if ! isinteractive()
    main()
end

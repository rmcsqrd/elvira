#!/home/elvira/programming/apps/julia/bin/julia

ENV["PYTHON"]="/usr/bin/python2.7"
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
@rosimport std_msgs.msg.String # note this imports as type StringMsg due to conflict
rostypegen()
using .geometry_msgs.msg
import .std_msgs.msg.StringMsg

function callback(msg::Pose2D, pub_obj::Publisher{Point})
    pt_msg = Point(msg.x, msg.y, 0.0)
    publish(pub_obj, pt_msg)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        text = StringMsg("epstein didn't kill himself")
        publish(pub_obj, text)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_example")
    pub = Publisher{StringMsg}("test", queue_size=10);
    sub = Subscriber{Pose2D}("pose", callback, (pub,), queue_size=10)
    loop(pub)
end

if ! isinteractive()
    main()
end

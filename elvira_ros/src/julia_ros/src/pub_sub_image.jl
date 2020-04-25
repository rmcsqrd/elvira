#!/home/elvira/programming/apps/julia/bin/julia

ENV["PYTHON"]="/usr/bin/python2.7"
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
@rosimport std_msgs.msg.String # note this imports as type StringMsg due to conflict
rostypegen()
using .geometry_msgs.msg
import .std_msgs.msg.StringMsg

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
    loop(pub)
end

if ! isinteractive()
    main()
end

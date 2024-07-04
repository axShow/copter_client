from client.modules.animation_parser import Animation
import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from pymavlink import mavutil
from mavros_msgs.srv import CommandLong
anim = Animation("animation.axsanim.yaml")
anim.load()

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
land = rospy.ServiceProxy('land', Trigger)
for frame in anim.frames:
    if frame.action == "land":
        land()
        print("landing")
        continue
    elif frame.action == "disarm":
        send_command(command=400, param2=21196) #force disarm
        print("disarming")
        continue
    else: 
        navigate(x=frame.x, y=frame.y, z=frame.z, 
             yaw=frame.yaw,
             frame_id=frame.nav_frame, 
             auto_arm=frame.action == "arm", 
             speed=1)
    print(frame.action, frame.x, frame.y, frame.z, frame.nav_frame)
    rospy.sleep(frame.delay)
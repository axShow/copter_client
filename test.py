from client.modules.animation_parser import Animation
import rospy
from clover import srv
from std_srvs.srv import Trigger

anim = Animation("animation.axsanim.yaml")
anim.load()

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
for frame in anim.frames:
    if frame.action == "land":
        land()
        print("landing")
        continue
    navigate(x=frame.x, y=frame.y, z=frame.z, 
             yaw=frame.yaw,
             frame_id=frame.nav_frame, 
             auto_arm=frame.action == "arm", 
             speed=1)
    print(frame.action, frame.x, frame.y, frame.z, frame.nav_frame)
    rospy.sleep(frame.delay)
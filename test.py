from client.modules.animation_parser import Animation
import rospy
from clover import srv

anim = Animation("animation.axsanim.yaml")
anim.load()

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
for frame in anim.frames:
    navigate(x=frame.x, y=frame.y, z=frame.z, frame=frame.nav_frame)
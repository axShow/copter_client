import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

navigate(x=0, y=0, z=0.5, speed=0.25, frame_id='terrain', auto_arm=True)
rospy.sleep(3)
navigate(y=0.5, speed=0.5, frame_id='body')
rospy.sleep(3)
navigate(y=-0.5, speed=0.5, frame_id='body')
rospy.sleep(3)
land()
rospy.sleep(3)
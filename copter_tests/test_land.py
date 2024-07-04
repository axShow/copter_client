import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import Mavlink
from mavros import mavlink

from pymavlink import mavutil
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State

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
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)


send_command(command=400, param2=21196) #force disarm
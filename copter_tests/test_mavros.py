from mavros_msgs.srv import ParamGet
from mavros_msgs.msg import ParamValue
import rospy
param_get = rospy.ServiceProxy('mavros/param/get', ParamGet)

# Считать параметр типа INT
value = param_get(param_id='MC_ROLLRATE_P').value

print(value)
def getParameters():
    
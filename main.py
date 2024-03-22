import random
import struct
import threading
import time
from time import sleep
import socket
import connector
from copterData import CopterData
from utils import send_msg, recv_msg
import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
rospy.init_node('my_node')
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
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def thread_2():
    while True:
        socket_r = connector.client
        if socket_r is not None:
            try:
                response = recv_msg(socket_r)
                if response is not None:
                    print(f'Received: "{response.decode("utf-16")}"')
                    act = response.decode("utf-16")
                    if act == 'land':
                        land()
                    elif act == 'take_off':
                        navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
                    elif act.startswith('led('):
                        params = list(map(int, act.replace("led(", "").replace(")", "").split(",")))
                        set_effect(r=params[0], g=params[1], b=params[2])

            except KeyboardInterrupt: break
            except OSError: pass



t: threading.Thread = threading.Thread(target=thread_2, daemon=True)
t.start()
while True:
    socket = connector.client
    if socket is not None:
        try:
            telem = get_telemetry()
            data = CopterData(name="clover1",
                              battery=telem.voltage,
                              controller_state=str(telem.connected),
                              flight_mode=telem.mode)
            message = data.model_dump_json()
            send_msg(socket, message.encode('utf-16'))
            print(f'Sending : "{message}"')

        except KeyboardInterrupt: break
        except OSError: pass
        except rospy.service.ServiceException: pass
    sleep(1)
connector.client.close()
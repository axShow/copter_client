import rospy
import os
import sys
import time
import math
from loguru import logger
import threading

# for backward compatibility with clever

from clover import srv

from sensor_msgs.msg import Range, Imu
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse,CommandLong
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../lib'))
sys.path.insert(0, lib_dir)

watchdog_is_enabled = True
log_state = True
vision_pose_delay_after_arm = 1.0
visual_pose_timeout = 2.0
pos_delta_max = 0.5
max_velocity = 5.0
max_tilt = 0.7 # radians, ~40 degrees
watchdog_action = 'emergency_land'
timeout_to_disarm = 5.0
emergency_land_thrust = 0.4
emergency_land_decrease_thrust_after = 0.5

logger.add('visual_pose_watchdog.log', level='INFO')

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_attitude = rospy.ServiceProxy('/set_attitude', srv.SetAttitude)
send_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

visual_pose_last_timestamp = 0
armed = False
mode = ''
laser_range = 10
emergency = False

local_pose = None
setpoint_raw = None
setpoint_position = None
setpoint_pose = None

arm_start_time = None
offboard_start_time = None
offboard_disarmed_timeout = 3.

linear_velocity = None
orientation = None
emergency_land_called = False

rospy.init_node('visual_pose_watchdog')
logger.info('visual_pose_watchdog inited')
logger.info('visual_pose_timeout = {} | position_delta_max = {} | watchdog_action = {}'.format(visual_pose_timeout, pos_delta_max, watchdog_action))
logger.info('timeout_to_disarm = {}'.format(timeout_to_disarm))
if watchdog_action == 'emergency_land':
    logger.info('emergency_land_thrust: {}'.format(emergency_land_thrust))

rate = rospy.Rate(10)

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def get_pos_delta(PoseStamped1, PoseStamped2):
    if PoseStamped1 is None or PoseStamped2 is None:
        return float('nan')
    pos1 = PoseStamped1.pose.position
    pos2 = PoseStamped2.pose.position
    return get_distance(pos1.x, pos1.y, pos1.z, pos2.x, pos2.y, pos2.z)

def get_time_delta(PoseStamped1, PoseStamped2):
    if PoseStamped1 is None or PoseStamped2 is None:
        return float('nan')
    time1 = PoseStamped1.header.stamp.to_sec()
    time2 = PoseStamped2.header.stamp.to_sec()
    return time1 - time2

def visual_pose_callback(data):
    global visual_pose_last_timestamp
    visual_pose_last_timestamp = data.header.stamp.to_sec()

def local_pose_callback(data):
    global local_pose
    local_pose = data

def setpoint_raw_callback(data):
    global setpoint_raw, setpoint_position, setpoint_pose
    setpoint_raw_pose = PoseStamped()
    setpoint_raw_pose.header = data.header
    setpoint_raw_pose.pose.position = data.position
    setpoint_raw = setpoint_raw_pose
    setpoint_pose = get_current_setpoint_pose(setpoint_raw, setpoint_position)

def setpoint_position_callback(data):
    global setpoint_raw, setpoint_position, setpoint_pose
    setpoint_position = data
    setpoint_pose = get_current_setpoint_pose(setpoint_raw, setpoint_position)

def get_current_setpoint_pose(_setpoint_raw, _setpoint_position):
    if _setpoint_position is None and _setpoint_raw is None:
        return None
    elif _setpoint_position is not None and _setpoint_raw is None:
        return _setpoint_position
    elif _setpoint_raw is not None and _setpoint_position is None:
        return _setpoint_raw
    else:
        return _setpoint_raw if _setpoint_raw.header.stamp > _setpoint_position.header.stamp else _setpoint_position

def state_callback(data):
    global armed, mode
    armed = data.armed
    mode = data.mode

def laser_callback(data):
    global laser_range
    laser_range = data.range

def velocity_callback(data):
    global linear_velocity
    linear_velocity = data.twist.linear

def orientation_callback(data):
    global orientation
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    orientation = (roll, pitch, yaw)

def failsafe_land():
    global emergency_land_thrust, laser_range, linear_velocity, orientation, armed, mode
    global emergency_land_decrease_thrust_after, timeout_to_disarm

    logger.info("Failsafe landing triggered")
    
    # Phase 1: Try AUTO.LAND
    phase = 'AUTO_LAND'
    logger.info("Phase 1: Attempting AUTO.LAND")
    
    try:
        set_mode(custom_mode='AUTO.LAND')
    except rospy.ServiceException as e:
        logger.warning(f"Could not set AUTO.LAND: {e}")

    start_time = time.time()
    emergency_start_time = None
    current_thrust = emergency_land_thrust
    
    while armed:
        now = time.time()
        
        # 1. Check Tilt (Priority 1)
        if orientation:
            roll, pitch, yaw = orientation
            # 40 degrees = 0.698 radians
            if abs(roll) > 0.7 or abs(pitch) > 0.7:
                logger.critical(f"Tilt limit exceeded (roll: {roll:.2f}, pitch: {pitch:.2f}). Disarming.")
                try:
                    send_command(command=400, param2=21196) # MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 to disarm
                except rospy.ServiceException: pass
                return

        # 2. Check Velocity (Switch to Phase 2)
        if phase == 'AUTO_LAND':
            vel_mag = 0.0
            if linear_velocity:
                vel_mag = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)
            
            if vel_mag > 5.0:
                logger.warning(f"Velocity uncontrolled ({vel_mag:.2f} m/s). Switching to Emergency Thrust.")
                phase = 'EMERGENCY_THRUST'
                emergency_start_time = now
            
            if mode != 'AUTO.LAND':
                 try:
                    set_mode(custom_mode='AUTO.LAND')
                 except rospy.ServiceException: pass

        # 3. Execute Phase 2
        if phase == 'EMERGENCY_THRUST':
            if emergency_start_time is None: emergency_start_time = now
            
            delta = now - emergency_start_time
            
            if (laser_range < 0.1 or delta > emergency_land_decrease_thrust_after) and current_thrust >= 0.:
                current_thrust -= 0.02
                if current_thrust < 0: current_thrust = 0
            
            logger.debug(f"Emergency thrust: {current_thrust:.2f}")
            try:
                set_attitude(thrust=current_thrust, yaw=0, frame_id='body', auto_arm=True)
            except rospy.ServiceException: pass
            
            if current_thrust <= 0:
                try:
                    send_command(command=400, param2=21196) # MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 to disarm
                except rospy.ServiceException: pass
        
        rate.sleep()

def emergency_land_service(request):
    global emergency_land_called, armed
    responce = TriggerResponse()
    if armed:
        responce.success = True
        responce.message = "Start emergency landing"
        emergency_land_called = True
    else:
        responce.success = False
        responce.message = "Copter is disarmed, no need for emergency landing!"
        emergency_land_called = False
    return responce

def watchdog_callback(event):
    global visual_pose_last_timestamp, armed, mode, watchdog_action, laser_range
    global emergency, local_pose, setpoint_pose, emergency_land_called, log_state
    global offboard_start_time, arm_start_time, vision_pose_delay_after_arm
    global linear_velocity, orientation

    pos_delta = get_pos_delta(local_pose, setpoint_pose)
    pos_dt = get_time_delta(local_pose, setpoint_pose)
    visual_pose_dt = abs(time.time() - visual_pose_last_timestamp)

    vel_mag = 0.0
    if linear_velocity:
        vel_mag = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)
    
    tilt_val = 0.0
    if orientation:
        tilt_val = max(abs(orientation[0]), abs(orientation[1]))

    if log_state:
        logger.info("armed: {} | mode: {} | vis_dt: {:.2f} | pos_delta: {:.2f} | vel: {:.2f} | tilt: {:.2f} | range: {:.2f}".format(
                    armed, mode, visual_pose_dt, pos_delta, vel_mag, tilt_val, laser_range))
    if mode == 'OFFBOARD':
        if offboard_start_time is None:
            offboard_start_time = time.time()
        if armed:
            if arm_start_time is None:
                arm_start_time = time.time()
            arm_time = time.time() - arm_start_time
            logger.debug('arm time: {}'.format(arm_time))
            if arm_time > vision_pose_delay_after_arm and watchdog_is_enabled:
                cond_vis = (visual_pose_dt > visual_pose_timeout and visual_pose_timeout != 0.)
                cond_pos = (pos_delta > pos_delta_max and pos_delta_max != 0.)
                cond_vel = (vel_mag > max_velocity and max_velocity != 0.)
                cond_tilt = (tilt_val > max_tilt and max_tilt != 0.)

                if cond_vis or cond_pos or cond_vel or cond_tilt:
                    action_timestamp = time.time()
                    emergency = True
                    
                    reasons = []
                    if cond_vis: reasons.append(f"Visual pose timeout ({visual_pose_dt:.2f} > {visual_pose_timeout})")
                    if cond_pos: reasons.append(f"Position delta too high ({pos_delta:.2f} > {pos_delta_max})")
                    if cond_vel: reasons.append(f"Velocity too high ({vel_mag:.2f} > {max_velocity})")
                    if cond_tilt: reasons.append(f"Tilt too high ({tilt_val:.2f} > {max_tilt})")

                    logger.info(f"Watchdog triggered: {', '.join(reasons)}. Starting failsafe landing sequence...")
                    failsafe_land()
                    logger.info('Disarmed')
                    emergency = False
            if emergency_land_called:
                emergency = True
                logger.info('/emergency_land service was called, start emergency landing...')
                failsafe_land()
                logger.info('Disarmed')
                emergency = False
                emergency_land_called = False
        else:
            arm_start_time = None
            if time.time() - offboard_start_time > offboard_disarmed_timeout:
                try:
                    set_mode(custom_mode='AUTO.LAND')
                except rospy.ServiceException as e:
                    logger.info(e)
    else:
        offboard_start_time = None
        if (abs(time.time() - visual_pose_last_timestamp) > visual_pose_timeout and visual_pose_timeout != 0.0):
            logger.info('Visual pose data is too old')

rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, visual_pose_callback)

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pose_callback)

rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, setpoint_position_callback)

rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, setpoint_raw_callback)

rospy.Subscriber('/mavros/state', State, state_callback)

rospy.Subscriber('/mavros/distance_sensor/rangefinder', Range, laser_callback)

rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocity_callback)

rospy.Subscriber('/mavros/imu/data', Imu, orientation_callback)
emergency_pub = rospy.Publisher('/emergency', Bool, queue_size=10)

rospy.Service('emergency_land', Trigger, emergency_land_service)

rospy.Timer(rospy.Duration(0.5), watchdog_callback)

while not rospy.is_shutdown():
    emergency_msg = Bool()
    emergency_msg.data = emergency
    emergency_pub.publish(emergency_msg)
    rate.sleep()
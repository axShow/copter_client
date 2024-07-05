#!/usr/bin/python
from __future__ import print_function

import asyncio
import math
import time
import threading
from loguru import logger
from typing_extensions import Tuple

from rospy import ROSException

try:
    import rospy # type: ignore
    from clover import srv # type: ignore
    from mavros_msgs.srv import SetMode # type: ignore
    from mavros_msgs.srv import CommandBool # type: ignore
    from std_srvs.srv import Trigger # type: ignore
    from pymavlink import mavutil # type: ignore
    from mavros_msgs.srv import CommandLong # type: ignore
    from sensor_msgs.msg import Range # type: ignore

    # create proxy service
    navigate = rospy.ServiceProxy("/navigate", srv.Navigate)
    set_position = rospy.ServiceProxy("/set_position", srv.SetPosition)
    set_rates = rospy.ServiceProxy("/set_rates", srv.SetRates)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
    arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    landing = rospy.ServiceProxy("/land", Trigger)
    emergency_land = rospy.ServiceProxy("/emergency_land", Trigger)
    send_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
except ImportError:
    import faker

    navigate = faker.navigate
    set_position = faker.set_position
    set_rates = faker.set_rates
    set_mode = faker.set_mode
    get_telemetry = faker.get_telemetry
    arming = faker.arming
    landing = faker.landing
    emergency_land = faker.emergency_land

services_list = [
    "/navigate",
    "/set_position",
    "/set_rates",
    "/mavros/set_mode",
    "/get_telemetry",
    "/mavros/cmd/arming",
    "/land",
    "/mavros/param/get",
]

logger.info("Proxy services inited")

# globals
FREQUENCY = 40  # HZ
TOLERANCE = 0.2
SPEED = 1.0
TAKEOFF_SPEED = 1.5
TIMEOUT = 5.0
TIMEOUT_ARMED = 2.0
TIMEOUT_DESCEND = TIMEOUT
TIMEOUT_LAND = 8.0
Z_DESCEND = 0.5
TAKEOFF_HEIGHT = 1.0
FRAME_ID = "map"
INTERRUPTER = threading.Event()
FLIP_MIN_Z = 0.5
KILL_Z = 0.15

checklist = []
get_telemetry_lock = threading.Lock()
delta = 0.0


def get_telemetry_locked(*args, **kwargs):
    with get_telemetry_lock:
        return get_telemetry(*args, **kwargs)


def arming_wrapper(state=False, *args, **kwargs):
    arming(state)


def kill_switch():
    send_command(command=400, param2=21196) #force disarm

async def force_land(kill_z=KILL_Z, timeout=TIMEOUT, freq=FREQUENCY, descend=False, timeout_descend=TIMEOUT_DESCEND):
    time_start = time.time()
    while True:
        if descend:
            logger.info("Descending to: | z: {:.3f}".format(0))
            # print("Descending to: | z: {:.3f}".format(z))
            await reach_altitude(
                z=0,
                timeout=timeout_descend,
                freq=freq,
                yaw=float("nan"),  # TODO yaw
            )
        try:
            dist = rospy.wait_for_message('rangefinder/range', Range, 30).range
        except ROSException: 
            logger.warning(
                    "Waiting rangefinder timed out! | time: 5 seconds"
                )
            return False, "Rangefinder timeout"
        if dist <= kill_z:
            logger.warning(f"Force disarming! | z: {dist:.3f}")
            kill_switch()
            return True, "success"

        time_passed = time.time() - time_start
        if timeout is not None:
            if time_passed >= timeout:
                logger.warning(
                    "Force landing timed out! | time: {:3f} seconds".format(
                        time_passed
                    )
                )
                return False, "Force land timeout"

        await asyncio.sleep(1/freq)
    

def interrupt():
    logger.info("Performing function interrupt")
    INTERRUPTER.set()


def get_distance3d(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def get_delta():
    global delta
    return delta


def reset_delta():
    global delta
    delta = 0


def navto(x, y, z, yaw=float("nan"), frame_id=FRAME_ID, auto_arm=False, **kwargs):
    global delta
    set_position(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, auto_arm=auto_arm)
    telemetry = get_telemetry_locked(frame_id=frame_id)
    delta = get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)

    logger.info(
        "Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}".format(x, y, z, yaw)
    )
    # logger.info('Delta: {}'.format(delta))
    # print('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    ##logger.info('Telemetry now: | z: {:.3f}'.format(telemetry.z))
    # print('Telemetry now: | z: {:.3f}'.format(telemetry.z))

    return True


def reach_point(
    x=0.0,
    y=0.0,
    z=0.0,
    yaw=float("nan"),
    speed=SPEED,
    tolerance=TOLERANCE,
    frame_id=FRAME_ID,
    auto_arm=False,
    freq=FREQUENCY,
    timeout=TIMEOUT,
    interrupter=INTERRUPTER,
    wait=False,
):
    rospy.loginfo(
        "Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}".format(
            x, y, z, yaw
        )
    )
    # print('Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed, auto_arm=auto_arm)

    # waiting for completion
    telemetry = get_telemetry_locked(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = time.time()

    while (
        get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z) > tolerance
    ) or wait:
        if interrupter.is_set():
            rospy.logwarn("Reach point function interrupted!")
            # print("Reach point function interrupted!")
            interrupter.clear()
            return False

        telemetry = get_telemetry_locked(frame_id=frame_id)
        rospy.logdebug(
            "Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}".format(
                telemetry.x, telemetry.y, telemetry.z, telemetry.yaw
            )
        )
        # print('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
        #    telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))
        rospy.logdebug(
            "Current delta: | {:.3f}".format(
                get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)
            )
        )
        # print('Current delta: | {:.3f}'.format(
        #    get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)))

        time_passed = time.time() - time_start

        if timeout is not None:
            if time_passed >= timeout:
                rospy.logwarn(
                    "Reaching point timed out! | time: {:3f} seconds".format(
                        time_passed
                    )
                )
                # print('Reaching point timed out! | time: {:3f} seconds'.format(time_passed))
                return wait
        rate.sleep()

    rospy.loginfo("Point reached!")
    # print("Point reached!")
    return True


async def reach_altitude(
    z=0.0,
    yaw=float("nan"),
    speed=SPEED,
    tolerance=TOLERANCE,
    frame_id=FRAME_ID,
    freq=FREQUENCY,
    timeout=TIMEOUT,
    interrupter=INTERRUPTER,
    wait=False,
):
    logger.info("Reaching attitude: | z: {:.3f} yaw: {:.3f}".format(z, yaw))
    # print('Reaching attitude: | z: {:.3f} yaw: {:.3f}'.format(z, yaw))
    current_telem = get_telemetry_locked(frame_id=frame_id)
    navigate(
        frame_id=frame_id,
        x=current_telem.x,
        y=current_telem.y,
        z=z,
        yaw=yaw,
        speed=speed,
    )

    telemetry = get_telemetry_locked(frame_id=frame_id)
    # rate = rospy.Rate(freq)
    time_start = time.time()

    while (abs(z - telemetry.z) > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Reach altitude function interrupted!")
            # print("Reach altitude function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry_locked(frame_id=frame_id)
        logger.info(
            "Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}".format(
                telemetry.x, telemetry.y, telemetry.z, telemetry.yaw
            )
        )
        # print('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
        #    telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))

        time_passed = time.time() - time_start
        if timeout is not None:
            if time_passed >= timeout:
                logger.warning(
                    "Reaching attitude timed out! | time: {:3f} seconds".format(
                        time_passed
                    )
                )
                # print('Reaching attitude timed out! | time: {:3f} seconds'.format(time_passed))
                return wait
        await asyncio.sleep(1 / freq)

    logger.info("Altitude reached!")
    # print("Altitude reached!")
    return True


def stop(frame_id="body", hold_speed=SPEED):
    navigate(frame_id=frame_id, yaw=float("nan"), speed=hold_speed)


async def land(
    descend=True,
    z=Z_DESCEND,
    frame_id_descend=FRAME_ID,
    frame_id_land=FRAME_ID,
    timeout_descend=TIMEOUT_DESCEND,
    timeout_land=TIMEOUT_LAND,
    freq=FREQUENCY,
    interrupter=INTERRUPTER,
) -> Tuple[bool, str]:
    reset_delta()
    if descend:
        logger.info("Descending to: | z: {:.3f}".format(z))
        # print("Descending to: | z: {:.3f}".format(z))
        reach_altitude(
            z=z,
            frame_id=frame_id_descend,
            timeout=timeout_descend,
            freq=freq,
            yaw=float("nan"),  # TODO yaw
            interrupter=interrupter,
        )
    landing()
    # print("Land is started")

    telemetry = get_telemetry_locked(frame_id=frame_id_land)
    # rate = rospy.Rate(freq)
    time_start = time.time()

    while telemetry.armed:
        if interrupter.is_set():
            logger.warning("Land function interrupted!")
            # print("Land function interrupted!")
            interrupter.clear()
            return False, "interrupted"

        telemetry = get_telemetry_locked(frame_id=frame_id_land)
        logger.info("Landing... | z: {}".format(telemetry.z))
        # print("Landing... | z: {}".format(telemetry.z))
        time_passed = time.time() - time_start

        if timeout_land is not None:
            if time_passed >= timeout_land:
                logger.warning(
                    "Landing timed out! | time: {:3f} seconds".format(time_passed)
                )
                logger.warning("Disarming!")
                # print("Landing timed out, disarming!!!")
                arming(False)
                return False, "timeout"
        await asyncio.sleep(1 / freq)

    logger.info("Landing succeeded!")
    # print("Landing succeeded!")
    return True, "success"


async def takeoff(
    height=TAKEOFF_HEIGHT,
    speed=TAKEOFF_SPEED,
    tolerance=TOLERANCE,
    frame_id=FRAME_ID,
    timeout_takeoff=TIMEOUT,
    interrupter=INTERRUPTER,
    emergency_land=False,
) -> Tuple[bool, str]:
    logger.info("Takeoff started...")
    # rate = rospy.Rate(FREQUENCY)
    start = get_telemetry_locked(frame_id=frame_id)
    climb = 0.0
    time_start = time.time()
    result = navigate(
        z=height, speed=speed, yaw=float("nan"), frame_id="body", auto_arm=True
    )
    # rospy.loginfo(result)
    if not result: #TODO: return to using result.success:
        return False, "not armed"
    # rospy.logdebug(result)
    logger.info("Takeoff to {:.2f} of {:.2f} meters".format(climb, height))
    while abs(climb - height) > tolerance:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return False, "interrupted"

        climb = abs(get_telemetry_locked(frame_id=frame_id).z - start.z)

        time_passed = time.time() - time_start

        if timeout_takeoff is not None:
            if time_passed >= timeout_takeoff:
                logger.info(
                    "Takeoff timed out! | time: {:3f} seconds".format(time_passed)
                )
                if emergency_land:
                    land(descend=False, interrupter=interrupter)
                return False, "timeout"

        # rate.sleep()
        await asyncio.sleep(1 / FREQUENCY)
    logger.info("Takeoff succeeded!")
    return True, "success"


async def flip(
    min_z=FLIP_MIN_Z, frame_id=FRAME_ID
) -> Tuple[bool, str]:  # TODO Flip in different directions
    logger.info("Flip started!")

    start_telemetry = get_telemetry_locked(
        frame_id=frame_id
    )  # memorize starting position

    if start_telemetry.z < min_z - TOLERANCE:
        logger.warning("Can't do flip! Flip failed!")
        # print("Can't do flip! Flip failed!")
        return False, "Height is not enough for flip"
    else:
        # Flip!
        set_rates(thrust=1)  # bump up
        await asyncio.sleep(0.2)

        set_rates(roll_rate=50, thrust=0.2)  # maximum roll rate
        timed = 0
        while True:
            telem = get_telemetry_locked()

            if abs(telem.roll) > math.pi / 2:
                break
            
            await asyncio.sleep(0.001)
            timed += 0.001
            if timed > 1.5:
                logger.warning("Flip timeout")
                return False, "roll changes timeouted"

        logger.info("Flip succeeded!")
        # print('Flip succeeded!')
        navto(
            x=start_telemetry.x,
            y=start_telemetry.y,
            z=start_telemetry.z,
            yaw=start_telemetry.yaw,
            frame_id=frame_id,
        )  # finish flip

        return True, "success"

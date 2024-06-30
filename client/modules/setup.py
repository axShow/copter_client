import socket
import subprocess
import xml.etree.ElementTree as ET
from typing_extensions import Literal, Union

from loguru import logger

from copterData import Coefficients, LPEFusion, TuneParams

cam_path = "/home/pi/catkin_ws/src/clover/clover/launch/main_camera.launch"
clover_path = "/home/pi/catkin_ws/src/clover/clover/launch/clover.launch"
aruco_path = "/home/pi/catkin_ws/src/clover/clover/launch/aruco.launch"
led_path = "/home/pi/catkin_ws/src/clover/clover/launch/led.launch"

try:
    import rospy
    import dynamic_reconfigure.client
    from mavros_msgs.srv import ParamSet, ParamGet
    from mavros_msgs.msg import ParamValue
    
    is_simulation = rospy.get_param('/use_sim_time', False)
    if is_simulation:
        aruco_path = aruco_path.replace("pi", "clover")
        clover_path = clover_path.replace("pi", "clover")
        led_path = led_path.replace("pi", "clover")
        cam_path = cam_path.replace("pi", "clover")
    param_set = rospy.ServiceProxy("mavros/param/set", ParamSet)
    param_get = rospy.ServiceProxy("mavros/param/get", ParamGet)
except ImportError:
    from faker import param_get, param_set
    from faker import ParamValueFake as ParamValue


def connect_wifi(ssid: str, password: str, hostname: Union[str, None]):
    logger.warning(subprocess.run(["client-setup", ssid, password,
                                   hostname if hostname is not None else socket.gethostname()]))


def run_setup(
    optical_flow: bool = False,
    rangefinder: bool = False,
    enable_aruco: bool = True,
    cam_direction: Literal["backward", "forward"] = "backward",
    setup_flight_controller: bool = False,
):
    main_camera = ET.parse(cam_path)
    clover = ET.parse(clover_path)
    aruco = ET.parse(aruco_path)
    led = ET.parse(led_path)
    for item in clover.getroot().iter():
        if item.tag != "arg":
            continue
        atr = item.attrib
        if atr.get("name") == "optical_flow":
            atr.update({"default": str(optical_flow).lower()})
        if atr.get("name") == "aruco":
            atr.update({"default": str(enable_aruco).lower()})
        if atr.get("name") == "rangefinder_vl53l1x":
            atr.update({"default": str(rangefinder).lower()})
    for item in aruco.getroot().iter():
        if item.tag != "arg":
            continue
        atr = item.attrib
        if atr.get("name") == "aruco_detect":
            atr.update({"default": str(enable_aruco).lower()})
        if atr.get("name") == "aruco_map":
            atr.update({"default": str(enable_aruco).lower()})
        if atr.get("name") == "aruco_vpe":
            atr.update({"default": str(enable_aruco).lower()})
        if (atr.get("name") == "map") and (item.tag == "arg"):
            atr.update({"default": "axshow_map.txt"})

    for item in main_camera.getroot().iter():
        if item.tag != "arg":
            continue
        atr = item.attrib
        if atr.get("name") == "direction_z":
            atr.update({"default": "down"})
        if atr.get("name") == "direction_y":
            atr.update({"default": cam_direction})

    main_camera.write(cam_path, short_empty_elements=False)
    clover.write(clover_path, short_empty_elements=False)
    aruco.write(aruco_path, short_empty_elements=False)
    if setup_flight_controller:
        param_set(param_id="SYS_MC_EST_GROUP", value=ParamValue(integer=1))
        param_set(
            param_id="LPE_FUSION", value=ParamValue(integer=20)
        )  # TODO: Change when use optical flow
        param_set(param_id="ATT_W_EXT_HDG", value=ParamValue(real=0.5))
        param_set(param_id="ATT_EXT_HDG_M", value=ParamValue(integer=1))
        param_set(param_id="LPE_VIS_XY", value=ParamValue(real=0.1))
        param_set(param_id="LPE_VIS_Z", value=ParamValue(real=0.1))
        param_set(param_id="LPE_VIS_DELAY", value=ParamValue(real=0))
    # with open('clover/launch/aruco.launch', 'wb') as f:
    #     aruco.write(f, encoding='utf-8')
    # with open('clover/launch/clover.launch', 'wb') as f:
    #     clover.write(f, encoding='utf-8')
    # with open('clover/launch/led.launch', 'wb') as f:
    #     led.write(f, encoding='utf-8')
    # with open('clover/launch/main_camera.launch', 'wb') as f:
    #     main_camera.write(f, encoding='utf-8')
def get_tune_params():
    coefficients_params = Coefficients.model_fields
    coefficients_values = {}
    for param_id, field in coefficients_params.items():
        param_type = field.annotation
        param = param_get(param_id=param_id)
        if not param.success: logger.error("Can't read parameter: " + param_id)
        if param_type == float:
            value = param.value.real
        else:
            value = param.value.integer
        coefficients_values.update({param_id: value})
    lpe_int = param_get(param_id="LPE_FUSION").value.integer
    coefficients = Coefficients.model_validate(coefficients_values)
    lpe_fusion = LPEFusion.fromInt(lpe_int)
    return TuneParams(lpe_fusion=lpe_fusion, coefficients=coefficients)

def set_tune_params(tune_params_dict: dict):
    tune_params = TuneParams.model_validate(tune_params_dict)
    coefficients = tune_params.coefficients.model_dump()
    lpe_int = LPEFusion.asInt(tune_params.lpe_fusion)
    for param_id, param_value in coefficients.items():
        param_type = type(param_value)
        new_value = ParamValue(real=param_value) if param_type == float else ParamValue(integer=param_value) 
        set = param_set(param_id=param_id, value=new_value)
        if not set.success: logger.error("Can't set parameter: " + param_id)
    param_set(param_id='LPE_FUSION', value=ParamValue(integer=lpe_int))
    return True


def generate_aruco_map(length: float = 0.3,
                 first: int = 0,
                 markers_x: int = 2,
                 markers_y: int = 2,
                 dist_x: int = 1,
                 dist_y: int = 1,
                 bottom_left: bool = False):
    # generating
    dynamic = False
    try:
        map_client = dynamic_reconfigure.client.Client('aruco_map', 5)
        dynamic = True
        paths = map_client.get_configuration().get("map", '/home/pi/catkin_ws/src/clover/aruco_pose/map/axshow_map.txt').split("/")
    except rospy.exceptions.ROSException:
        paths = '/home/pi/catkin_ws/src/clover/aruco_pose/map/axshow_map.txt'.split("/")
    if is_simulation:
        paths[paths.index("pi")] = "clover"
    directory = "/".join(paths[0:len(paths)-1])
    output = open(directory + "/axshow_map.txt", 'w+')
    max_y = (markers_y - 1) * dist_y
    output.write('# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x\n')
    for y in range(markers_y):
        for x in range(markers_x):
            pos_x = x * dist_x
            pos_y = y * dist_y
            if not bottom_left:
                pos_y = max_y - pos_y
            output.write('{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(first, length, pos_x, pos_y, 0, 0, 0, 0))
            first += 1

    # applying in runtime
    if dynamic: map_client.update_configuration({'map': directory + '/axshow_map.txt'})
    # write changes in config files
    aruco = ET.parse(aruco_path)
    for item in aruco.getroot().iter():
        if item.tag != "arg":
            continue
        atr = item.attrib
        if (atr.get("name") == "map") and (item.tag == "arg"):
            atr.update({"default": "axshow_map.txt"})
    aruco.write(aruco_path, short_empty_elements=False)
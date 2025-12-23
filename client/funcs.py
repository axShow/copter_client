from typing_extensions import Literal, Union
from copterData import TuneParams
from modules.animation_processor import run_animation
from modules.animation_processor import INTERRUPTER as show_interrupter
from modules.flight import *
from modules import led, setup
from modules.other import *
from modules.setup import connect_wifi, get_tune_params, set_tune_params, generate_aruco_map
from fastapi import APIRouter, UploadFile


functions = {}

router = APIRouter()


@router.post("/land")
async def land_wrap(descend=True,
    z=Z_DESCEND):
    res, details = await land(descend=descend, z=z)
    return {"result": res, "details": details}


@router.post("/takeoff")
async def takeoff_wrap(height=TAKEOFF_HEIGHT,
    speed=TAKEOFF_SPEED,):
    res, details = await takeoff(height=height, speed=speed)
    return {"result": res, "details": details}


@router.post("/rth")
async def rth_wrap(height=None):
    res, details = await reach_point(x=home_point[0], y=home_point[1], z=height if height is not None else home_point[2],
        frame_id="map")
    return {"result": res, "details": details}


@router.post("/led")
async def led_wrapper(r: int = 0, g: int = 0, b: int = 0,
    effect: led.LEDEffects = led.LEDEffects.FILL):
    res, details = led.set_effect(r=r, g=g, b=b, effect=effect)
    logger.info(f"LED set to {r}, {g}, {b} with effect {effect}")   
    return {"result": res, "details": details}


@router.post("/setup")
async def setup(optical_flow: bool = False,
    rangefinder: bool = False,
    enable_aruco: bool = True,
    cam_direction: Literal["backward", "forward"] = "backward",
    setup_flight_controller: bool = False):
    setup.run_setup(optical_flow=optical_flow,
                    rangefinder=rangefinder,
                    enable_aruco=enable_aruco,
                    cam_direction=cam_direction,
                    setup_flight_controller=setup_flight_controller)
    return {"result": True, "details": "success"}


@router.post("/set_arming")
async def set_arming(state: bool):
    arming_wrapper(state=state)
    return {"result": True, "details": "success"}

@router.post("/kill_switch")
async def kill_switch():
    await kill_switch()
    return {"result": True, "details": "success"}


@router.post("/flip")
async def flip_wrapper(min_z=FLIP_MIN_Z):
    res, details = await flip(min_z=min_z)
    return {"result": res, "details": details}


@router.post("/calibrate_gyro")
async def calibrate_gyro_wrapper():
    details = calibrate_gyro()
    return {"result": True, "details": details}


@router.post("/calibrate_level")
async def calibrate_level_wrapper():
    details = calibrate_level()
    return {"result": True, "details": details}


@router.post("/file_transfer")
async def file_transfer_wrapper(destination: str, file: UploadFile):
    contents = await file.read()
    file_transfer(destination=destination, data=contents)
    return {"result": True, "details": "success"}


@router.post("/upload_animation")
async def upload_animation_wrapper(file: UploadFile):
    contents = await file.read()
    result = upload_animation(data=contents.decode("utf-8"))
    return {"result": result[0], "details": result[1]}


@router.post("/connect_wifi")
async def connect_wifi_wrapper(ssid: str, password: str, hostname: Union[str, None] = None):
    connect_wifi(ssid=ssid, password=password, hostname=hostname)
    return {"result": True, "details": "connecting"}


@router.post("/generate_aruco_map")
async def generate_map(length: float = 0.3,
                 first: int = 0,
                 markers_x: int = 2,
                 markers_y: int = 2,
                 dist_x: int = 1,
                 dist_y: int = 1,
                 bottom_left: bool = False):
    generate_aruco_map(length=length,
                       first=first,
                       markers_x=markers_x,
                       markers_y=markers_y,
                       dist_x=dist_x,
                       dist_y=dist_y,
                       bottom_left=bottom_left)
    return {"result": True, "details": "success"}


@router.post("/reboot_fcu")
async def reboot_fcu_wrapper():
    reboot_fcu()
    return {"result": True, "details": "success"}


@router.post("/reboot_system")
async def reboot_system_wrapper():
    reboot_system()
    return {"result": True, "details": "success"}


@router.post("/restart_service")
async def restart_service_wrapper():
    restart_service()
    return {"result": True, "details": "success"}


@router.post("/restart_clover")
async def restart_clover_wrapper():
    restart_clover()
    return {"result": True, "details": "success"}


@router.post("/kill_client")
async def kill_client_wrapper():
    stop_service()
    return {"result": True, "details": "success"}


# @command()
# async def self_check(args: dict):
#     selfcheck()
#     return {"result": True, "details": "success"}


@router.post("/run_show")
async def run_show(start_ts, offset):
    await run_animation(start_ts=start_ts, offset=offset)
    return {"result": True, "details": "success"}


@router.get("/tune_params", response_model=TuneParams)
async def get_tune_params_wrapper():
    values = get_tune_params()
    return values


@router.post("/tune_params")
async def set_tune_params_wrapper(params: TuneParams):
    result = set_tune_params(params)
    return {"result": result, "details": "success" if result else "failed"}


@router.post("/interrupt_show")
async def interrupt_show():
    show_interrupter.set()
    return {"result": True, "details": "success"}

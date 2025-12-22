from modules.animation_processor import run_animation
from modules.animation_processor import INTERRUPTER as show_interrupter
from modules.flight import *
from modules import led, setup
from modules.other import *
from modules.setup import connect_wifi, get_tune_params, set_tune_params, generate_aruco_map
from functools import wraps
import connector
# async def proccess(method: str, args: dict) -> dict:
#     if method == "land":
#         # ARGS:
#         # descend = (True,)
#         # z = (Z_DESCEND,)
#         # frame_id_descend = (FRAME_ID,)
#         # frame_id_land = (FRAME_ID,)
#         # timeout_descend = (TIMEOUT_DESCEND,)
#         # timeout_land = (TIMEOUT_LAND,)
#         # freq = (FREQUENCY,)
#         # interrupter = INTERRUPTER
#         res, details = await land(**args)
#         return {"result": res, "details": details}
#     elif method == "emergency_land":
#         # ARGS:
#         # descend = (True,)
#         # z = (Z_DESCEND,)
#         # frame_id_descend = (FRAME_ID,)
#         # frame_id_land = (FRAME_ID,)
#         # timeout_descend = (TIMEOUT_DESCEND,)
#         # timeout_land = (TIMEOUT_LAND,)
#         # freq = (FREQUENCY,)
#         # interrupter = INTERRUPTER
#         res = emergency_land(**args)
#         return {"result": True, "details": str(res)}
#     elif method == "takeoff":
#         # ARGS
#         # height = (TAKEOFF_HEIGHT,)
#         # speed = (TAKEOFF_SPEED,)
#         # tolerance = (TOLERANCE,)
#         # frame_id = (FRAME_ID,)
#         # timeout_takeoff = (TIMEOUT,)
#         # interrupter = (INTERRUPTER,)
#         # emergency_land = (False,)
#         res, details = await takeoff(**args)
#         return {"result": res, "details": details}
#     elif method == "led":
#         # ARGS
#         # r 0-255
#         # g 0-255
#         # b 0-255
#         # effect =
#         # fill (или пустая строка) – залить всю ленту цветом;
#         # blink – мигание цветом;
#         # blink_fast – ускоренное мигание цветом;
#         # fade – плавное перетекание в цвет;
#         # wipe – "надвигание" нового цвета;
#         # flash – быстро мигнуть цветом 2 раза и вернуться к предыдущему эффекту;
#         # rainbow – переливание ленты цветами радуги;
#         # rainbow_fill – переливать заливку по цветам радуги.
#         res, details = led.set_effect(**args)
#         return {"result": res, "details": details}
#     elif method == "setup":
#         # optical_flow: bool = (False,)
#         # rangefinder: bool = (False,)
#         # enable_aruco: bool = (True,)
#         # cam_direction: Literal["backward", "forward"] = "backward"
#         # setup_flight_controller: bool = False
#         setup.run_setup(**args)
#         return {"result": True, "details": "Unknown"}
#     elif method == "set_arming":
#         # state: bool
#         arming_wrapper(**args)
#         return {"result": True, "details": "Unknown"}
#     elif method == "flip":
#         # min_z: float
#         # frame_id
#         res, details = await flip(**args)
#         return {"result": res, "details": details}
#     elif method == "calibrate_gyro":
#         details = calibrate_gyro()
#         return {"result": True, "details": details}
#     elif method == "calibrate_level":
#         details = calibrate_level()
#         return {"result": True, "details": details}
#     elif method == "file_transfer":
#         # ARGS
#         # destination: str
#         # data: bytes
#         file_trans(**args)
#     elif method == "connect_wifi":
#         # ARGS
#         # ssid: str
#         # password: str
#         # hostname: str
#         connect_wifi(**args)
#         return {"result": True, "details": "connecting"}
#     elif method == "generate_map":
#         # ARGS
#         # size: float
#         # dist_x: float
#         # dist_y: float
#         # x_num: int
#         # y_num: int
#         # bottom_left: bool
#         # start_id: int
#         pass
#     elif method == "reboot_fcu":
#         reboot_fcu()
#         return {"result": True, "details": "success"}
#     elif method == "reboot_system":
#         reboot_system()
#         return {"result": True, "details": "success"}
#     elif method == "restart_client":
#         restart_service()
#         return {"result": True, "details": "success"}
#     elif method == "restart_clover":
#         restart_clover()
#         return {"result": True, "details": "success"}
#     elif method == "kill_client":
#         stop_service()
#         return {"result": True, "details": "success"}
#     elif method == "self_check":
#         selfcheck()
#         return {"result": True, "details": "success"}
#     elif method == "get_tune_params":
#         values = get_tune_params()
#         return {"result": True, "details": "success", "payload": values.model_dump()}
#     elif method == "set_tune_params":
#         result = set_tune_params(args)
#         return {"result": result, "details": "Unknown"}
#     else:
#         return {"result": False, "details": "command not found"}


functions = {}


def command(alias=None):
    def decorator(func):
        @wraps(func)  # Preserves original function metadata
        async def wrapper(*args, **kwargs):
            return await func(*args, **kwargs) if asyncio.iscoroutinefunction(func) else func(*args, **kwargs)

        wrapper.original_func = func
        functions[alias if alias else func.__name__] = wrapper
        return wrapper

    return decorator


@command(alias="land")
async def land_wrap(args: dict):
    res, details = await land(**args)
    return {"result": res, "details": details}


@command(alias="takeoff")
async def takeoff_wrap(args: dict):
    res, details = await takeoff(**args)
    return {"result": res, "details": details}


@command(alias="led")
async def led_wrapper(args: dict):
    res, details = led.set_effect(**args)
    return {"result": res, "details": details}


@command()
async def setup(args: dict):
    setup.run_setup(**args)
    return {"result": True, "details": "Unknown"}


@command()
async def set_arming(args: dict):
    arming_wrapper(**args)
    return {"result": True, "details": "Unknown"}


@command(alias="flip")
async def flip_wrapper(args: dict):
    res, details = await flip(**args)
    return {"result": res, "details": details}


@command(alias="calibrate_gyro")
async def calibrate_gyro_wrapper(args: dict):
    details = calibrate_gyro()
    return {"result": True, "details": details}


@command(alias="calibrate_level")
async def calibrate_level_wrapper(args: dict):
    details = calibrate_level()
    return {"result": True, "details": details}


@command(alias="file_transfer")
async def file_transfer_wrapper(args: dict):
    file_trans(**args)


@command(alias="upload_animation")
async def upload_animation_wrapper(args: dict):
    result = upload_animation(**args)
    return {"result": result[0], "details": result[1]}


@command(alias="connect_wifi")
async def connect_wifi_wrapper(args: dict):
    connect_wifi(**args)
    return {"result": True, "details": "connecting"}


@command()
async def generate_map(args: dict):
    generate_aruco_map(**args)
    return {"result": True, "details": "success"}


@command(alias="reboot_fcu")
async def reboot_fcu_wrapper(args: dict):
    reboot_fcu()
    return {"result": True, "details": "success"}


@command(alias="reboot_system")
async def reboot_system_wrapper(args: dict):
    reboot_system()
    return {"result": True, "details": "success"}


@command()
async def restart_client(args: dict):
    restart_service()
    return {"result": True, "details": "success"}


@command(alias="restart_clover")
async def restart_clover_wrapper(args: dict):
    restart_clover()
    return {"result": True, "details": "success"}


@command()
async def kill_client(args: dict):
    stop_service()
    return {"result": True, "details": "success"}


@command()
async def self_check(args: dict):
    selfcheck()
    return {"result": True, "details": "success"}


@command()
async def run_show(args: dict):
    await run_animation(**args, offset=connector.time_offset)
    return {"result": True, "details": "success"}


@command(alias="get_tune_params")
async def get_tune_params_wrapper(args: dict):
    values = get_tune_params()
    return {"result": True, "details": "success", "payload": values.model_dump()}


@command(alias="set_tune_params")
async def set_tune_params_wrapper(args: dict):
    result = set_tune_params(args)
    return {"result": result, "details": "Unknown"}


@command()
async def interrupt_show(args: dict):
    show_interrupter.set()
    return {"result": True, "details": "Unknown command"}

@command()
async def default(args: dict):
    return {"result": False, "details": "Unknown command"}


async def execute_method(method: str, args: dict) -> dict:
    return await functions.get(method, default)(args)

from modules import led, setup
from modules.flight import *
from modules.other import calibrate_gyro, calibrate_level, file_trans


def proccess(method: str, args: dict) -> dict:
    if method == "land":
        # ARGS:
        # descend = (True,)
        # z = (Z_DESCEND,)
        # frame_id_descend = (FRAME_ID,)
        # frame_id_land = (FRAME_ID,)
        # timeout_descend = (TIMEOUT_DESCEND,)
        # timeout_land = (TIMEOUT_LAND,)
        # freq = (FREQUENCY,)
        # interrupter = INTERRUPTER
        res, details = land(**args)
        return {"result": res, "details": details}
    elif method == "takeoff":
        # ARGS
        # height = (TAKEOFF_HEIGHT,)
        # speed = (TAKEOFF_SPEED,)
        # tolerance = (TOLERANCE,)
        # frame_id = (FRAME_ID,)
        # timeout_takeoff = (TIMEOUT,)
        # interrupter = (INTERRUPTER,)
        # emergency_land = (False,)
        res, details = takeoff(**args)
        return {"result": res, "details": details}
    elif method == "led":
        # ARGS
        # r 0-255
        # g 0-255
        # b 0-255
        # effect =
        # fill (или пустая строка) – залить всю ленту цветом;
        # blink – мигание цветом;
        # blink_fast – ускоренное мигание цветом;
        # fade – плавное перетекание в цвет;
        # wipe – "надвигание" нового цвета;
        # flash – быстро мигнуть цветом 2 раза и вернуться к предыдущему эффекту;
        # rainbow – переливание ленты цветами радуги;
        # rainbow_fill – переливать заливку по цветам радуги.
        res, details = led.set_effect(**args)
        return {"result": res, "details": details}
    elif method == "setup":
        # optical_flow: bool = (False,)
        # rangefinder: bool = (False,)
        # enable_aruco: bool = (True,)
        # cam_direction: Literal["backward", "forward"] = "backward"
        setup.run_setup(**args)
        return {"result": True, "details": "Unknown"}
    elif method == "set_arming":
        # state: bool
        arming_wrapper(**args)
        return {"result": True, "details": "Unknown"}
    elif method == "flip":
        # min_z: float
        # frame_id
        res, details = flip(**args)
        return {"result": res, "details": details}
    elif method == "calibrate_gyro":
        details = calibrate_gyro()
        return {"result": True, "details": details}
    elif method == "calibrate_level":
        details = calibrate_level()
        return {"result": True, "details": details}
    elif method == "file_transfer":
        # ARGS
        # destination: str
        # data: bytes
        file_trans(**args)
    elif method == "connect_wifi":
        # ARGS
        # ssid: str
        # password: str
        # hostname: str
        pass
    elif method == "generate_map":
        # ARGS
        # size: float
        # dist_x: float
        # dist_y: float
        # x_num: int
        # y_num: int
        # bottom_left: bool
        # start_id: int
        pass
    else:
        return {"result": False, "details": "command not found"}

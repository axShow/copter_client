import random
import logging

from pydantic import BaseModel, Field
from copterData import TelemetryData, FlightMode
logger = logging.getLogger(__name__)
data = TelemetryData(voltage=0, mode=FlightMode.MANUAL, armed=False)
def get_telemetry(*args, **kwargs):
    data.voltage = random.uniform(13.2, 16.8)
    return data


def navigate(*args, **kwargs):
    logger.info(f"Navigating, parameters: {args}, {kwargs}")
    try:
        if kwargs["auto_arm"]:
            arming(True)
    except KeyError:
        pass
    try:
        data.x = kwargs["x"]
    except KeyError:
        pass
    try:
        data.y = kwargs["y"]
    except KeyError:
        pass
    try:
        data.z = kwargs["z"]
    except KeyError:
        pass
    return True


def set_position(*args, **kwargs):
    logger.info(f"Setting position, parameters: {args}, {kwargs}")
    try:
        if kwargs["auto_arm"]:
            arming(True)
    except KeyError:
        pass
    try:
        data.x = kwargs["x"]
    except KeyError:
        pass
    try:
        data.y = kwargs["y"]
    except KeyError:
        pass
    try:
        data.z = kwargs["z"]
    except KeyError:
        pass
    return True


def set_rates(*args, **kwargs):
    logger.info(f"Setting rates, parameters: {args}, {kwargs}")
    return None


def set_mode(*args, **kwargs):
    logger.info(f"Setting mode, parameters: {args}, {kwargs}")
    return None


def arming(state: bool, *args, **kwargs):
    logger.info(f"Arming mode, parameters: {state}, {args}, {kwargs}")
    data.armed = state


def landing(*args, **kwargs):
    logger.info("Landing, parameters: {args}, {kwargs}")
    data.z = 0
    arming(False)
    return True


def emergency_land(*args, **kwargs):
    logger.info("Emergency Landing, parameters: {args}, {kwargs}")
    return None


def set_effect_service(*args, **kwargs):
    global data
    logger.info("Setting led efffect, parameters: {args}, {kwargs}")
    data.r, data.g, data.b = kwargs.get("r", 0), kwargs.get("g", 0), kwargs.get("b", 0)
    return None

float_type = float

class ValueFake(BaseModel):
    integer: int = Field(0)
    real: float_type = Field(0.0)

class ParamValueFake(BaseModel):
    success: bool = Field(True)
    value: ValueFake


def param_set(param_id: str, value: ValueFake):
    logger.warning(f"SettedParam: {param_id}={value.model_dump(exclude_unset=True)}")
    return ParamValueFake(success=True, value=value)

def param_get(param_id: str):
    return ParamValueFake(value=ValueFake(integer=1, real=0.5))
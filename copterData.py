from enum import Enum
from typing import Union

from pydantic import BaseModel, Field


class CopterData(BaseModel):
    type: str = Field("Info")
    name: str
    battery: float
    flight_mode: str
    controller_state: str


class Query(BaseModel):
    id: int
    method_name: str
    args: dict


class Response(BaseModel):
    type: str = Field("Response")
    id: int
    result: dict


class FlightMode(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    OFFBOARD = "OFFBOARD"
    ACRO = "ACRO"


class TelemetryData(BaseModel):
    voltage: float
    connected: Union[bool, str] = Field("VIRTUAL")
    armed: bool

    # velocities
    vx: float = (Field(0),)
    vy: float = (Field(0),)
    vz: float = (Field(0),)

    # rates (angular velocity)
    yaw_rate: float = Field(0)
    roll_rate: float = Field(0)
    pitch_rate: float = Field(0)

    # angles
    yaw: float = Field(0)
    pitch: float = Field(0)
    roll: float = Field(0)

    # positions
    x: float = Field(0)
    y: float = Field(0)
    z: float = Field(0)

    mode: FlightMode

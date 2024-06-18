from enum import Enum
from typing_extensions import Union

from pydantic import BaseModel, Field

class Coefficients(BaseModel):
    MC_ROLLRATE_P: float
    MC_ROLLRATE_I: float
    MC_ROLLRATE_D: float
    MC_PITCHRATE_P: float
    MC_PITCHRATE_I: float
    MC_PITCHRATE_D: float
    MPC_XY_VEL_P: float
    MPC_Z_VEL_P: float
    MPC_THR_HOVER: float
# print(Coefficents.model_fields.get("MC_ROLLRATE_P").annotation == int)
class LPEFusion(BaseModel):
    GPS: bool = Field(False)
    OpticalFlow: bool = Field(False)
    VisionPosition: bool = Field(False)
    LandingTarget: bool = Field(False)
    LandDetector: bool = Field(False)
    PubAglAsLposDown: bool = Field(False)
    FlowGyroCompensation: bool = Field(False)
    Baro: bool = Field(False)

    def asInt(self):
        data = [self.GPS, self.OpticalFlow, self.VisionPosition,
        self.LandingTarget, self.LandDetector, self.PubAglAsLposDown, self.FlowGyroCompensation,
        self.Baro]
        data_int = map(int, data)
        byte = "".join(map(str, data_int))
        return int(byte[::-1], 2)

    @staticmethod
    def fromInt(value):
        data_int = list(str(bin(value)))[2:]
        data = list(reversed(list(map(bool, map(int, data_int)))))
        self = LPEFusion().model_dump()
        for param, value in zip(self.keys(), data):
            self.update({param: value})
        return LPEFusion().model_validate(self)

class TuneParams(BaseModel):
    lpe_fusion: LPEFusion
    coefficients: Coefficients

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

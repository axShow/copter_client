from typing_extensions import Literal, List
from pydantic import BaseModel, Field
import yaml

NavFrame = Literal["body", "aruco_map", "map", "gps"]
MoveType = Literal["relative", "absolute"]
StartAction = Literal["auto", "takeoff", "fly"]
CordsSystem = Literal["global", "local"]


class Frame(object):
    nav_frame: NavFrame = "map"
    x: float = None
    y: float = None
    z: float = None
    yaw: float = float("nan")
    r: int = 0
    g: int = 0
    b: int = 0

    def __init__(self, row: str, nav_frame: NavFrame=nav_frame):
        data = row.split(",")
        self.nav_frame = nav_frame
        self.x, self.y, self.z, self.yaw, self.r, self.g, self.b = data[1:8]

class Config(BaseModel):
    move_type: MoveType = Field("absolute")
    nav_frame: NavFrame = Field("map")
    frame_delay: float = Field(0.1)
    start_action: StartAction = Field("auto")
    cords_system: CordsSystem = Field("global")

class Animation(object):
    frames: List[Frame] = []
    filepath = None
    config: Config = None

    def __init__(self, animation_path: str):
        self.filepath = animation_path

    # async def execute_frames():
    #     for frame in frames:
    #         navigate()

    def load(self):
        with open(self.filepath, 'r', encoding='utf8') as stream:
            data = stream.read().split("==================================\n")
            self.config = Config.model_validate(yaml.safe_load(data[0])["config"])
            raw_frames = data[1].split("\n")
            for rframe in raw_frames[1:]:
                self.frames.append(Frame(rframe, "body"))





from typing_extensions import Literal, List
from pydantic import BaseModel, Field
import yaml

NavFrame = Literal["body", "aruco_map", "map", "gps"]
MoveType = Literal["relative", "absolute"]
StartAction = Literal["takeoff", "fly"]
EndAction = Literal["stay", "land", "disarm"]
FrameAction = Literal["arm", "route", "land", "disarm"]
CordsSystem = Literal["global", "local"]


class Frame(object):
    nav_frame: NavFrame = "map"
    x: float = None
    y: float = None
    z: float = None
    yaw: float = float("nan")
    action: FrameAction = "route"
    r: int = 0
    g: int = 0
    b: int = 0
    delay: float = 0.1

    def __init__(self, row: str, nav_frame: NavFrame=nav_frame, delay=0.1, action: FrameAction = action):
        data = row.split(",")
        self.nav_frame = nav_frame
        self.action = action
        self.delay = delay
        self.x, self.y, self.z, self.yaw = map(float, data[1:5])
        self.r, self.g, self.b = map(int, data[5:8])

class Config(BaseModel):
    move_type: MoveType = Field("absolute")
    nav_frame: NavFrame = Field("map")
    frame_delay: float = Field(0.1)
    start_action: StartAction = Field("fly")
    end_action: EndAction = Field("land")
    cords_system: CordsSystem = Field("global")
    takeoff_height: float = Field(1)
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
            self.frames = []
            data = stream.read().split("==================================\n")
            self.config = Config.model_validate(yaml.safe_load(data[0])["config"])
            nav_frame = self.config.nav_frame
            raw_frames = data[1].split("\n")
            if (self.config.start_action == "takeoff"):
                self.frames.append(Frame(f"-1,0,0,{self.config.takeoff_height},nan,0,0,0", "body", 5, "arm"))
            for rframe in raw_frames[1:]:
                self.frames.append(Frame(rframe, nav_frame, self.config.frame_delay))
            if (self.config.end_action == "land"):
                self.frames.append(Frame(f"999,0,0,0,nan,0,0,0", action="land", delay=5))
                self.frames.append(Frame(rframe, nav_frame, self.config.frame_delay))
            if (self.config.end_action == "disarm"):
                self.frames.append(Frame(f"999,0,0,0,nan,0,0,0", action="disarm", delay=5))
            self.frames[0].action = "arm"





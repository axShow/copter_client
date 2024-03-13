from pydantic import BaseModel


class CopterData(BaseModel):
    name: str
    battery: float
    flight_mode: str
    controller_state: str
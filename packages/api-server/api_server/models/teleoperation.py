from pydantic import BaseModel


class VideoRoomModel(BaseModel):
    url: str


class MotionObjectModel(BaseModel):
    # All values are relative to current position
    # foward direction is positive
    # clockwise direction is positive
    name: str
    x_m: float
    y_m: float
    z_m: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

from pydantic import BaseModel


class StringModel(BaseModel):
    data: str


class FloatModel(BaseModel):
    data: float


class CmdVelModel(BaseModel):
    robot_id: str
    linear_x: float
    linear_y: float
    linear_z: float
    rotation_x: float
    rotation_y: float
    rotation_z: float

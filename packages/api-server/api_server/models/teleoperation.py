from pydantic import BaseModel


class VideoRoomModel(BaseModel):
    target_name: str
    url: str

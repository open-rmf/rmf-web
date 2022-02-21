from pydantic import BaseModel


class VideoRoomModel(BaseModel):
    url: str

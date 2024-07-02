from pydantic import BaseModel


class BeaconState(BaseModel):
    id: str
    online: bool
    category: str
    activated: bool
    level: str

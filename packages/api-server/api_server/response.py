from fastapi.responses import PlainTextResponse


class RawJSONResponse(PlainTextResponse):
    media_type = "application/json"

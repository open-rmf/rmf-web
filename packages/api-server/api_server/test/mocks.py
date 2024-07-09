from contextlib import contextmanager
from unittest.mock import AsyncMock, MagicMock, patch

from api_server.app import app


@contextmanager
def patch_sio():
    """
    Mocks the socketio server used by the app.
    """
    with patch.object(app, "sio") as mock_sio:
        session = {}
        mock_sio.get_session = AsyncMock(return_value=session)
        session_context = MagicMock()
        session_context.__aenter__ = AsyncMock(return_value=session)
        mock_sio.session.return_value = session_context
        yield mock_sio

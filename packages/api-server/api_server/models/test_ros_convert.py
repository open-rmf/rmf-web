import unittest

from builtin_interfaces.msg import Time
from rmf_door_msgs.msg import DoorMode, DoorState

from .ros_convert import update_message_from_dict


class TestRosConvert(unittest.TestCase):
    def test_convert_time(self):
        time = Time()
        update_message_from_dict(time, {"sec": 1, "nanosec": 2})
        self.assertEqual(time.sec, 1)
        self.assertEqual(time.nanosec, 2)

    def test_convert_nested(self):
        state = DoorState()
        update_message_from_dict(
            state,
            {
                "door_time": {"sec": 1, "nanosec": 2},
                "door_name": "test_door",
                "current_mode": {"value": DoorMode.MODE_OPEN},
            },
        )
        self.assertEqual(state.door_name, "test_door")
        self.assertEqual(state.door_time.sec, 1)
        self.assertEqual(state.door_time.nanosec, 2)
        self.assertEqual(state.current_mode.value, DoorMode.MODE_OPEN)

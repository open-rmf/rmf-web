# Temporary message definition file for eventual port to rmf_api_msgs(?)


class TeleoperationMessages:
    TELEOPERATION_JOIN_VIDEO_ROOM_DEFINITION = {"robot_id": "", "room_id": ""}

    TELEOPERATION_LEAVE_VIDEO_ROOM_DEFINITION = {"robot_id": "", "room_id": ""}

    TELEOPERATION_DRIVE_ROBOT_IN_METERS_FORWARD_DEFINITION = {
        "robot_id": "",
        "x_m": 0.0,
        "x_vel_m_s": 0.0,
    }

    TELEOPERATION_ROTATE_ROBOT_IN_RADIANS_CLOCKWISE_DEFINITION = {
        "robot_id": "",
        "theta_rad": 0.0,
        "theta_vel_rad_s": 0.0,
    }

    TELEOPERATION_CMD_VEL_DEFINITION = {
        "robot_id": "",
        "linear_x": 0.0,
        "linear_y": 0.0,
        "linear_z": 0.0,
        "rotation_x": 0.0,
        "rotation_y": 0.0,
        "rotation_z": 0.0,
    }

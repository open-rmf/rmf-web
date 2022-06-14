from .ros import ros_node


def now() -> int:
    """
    Return current unix time in millis
    """
    ros_time = ros_node().get_clock().now()
    return ros_time.nanoseconds // 1000000

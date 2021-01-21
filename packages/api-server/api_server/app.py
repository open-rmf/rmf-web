import base64
import hashlib
import os
import signal
import threading
from urllib.parse import urljoin

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict

from flask import Flask

from .building_map import make_blueprint


class MainNode(Node):
    def __init__(self):
        super().__init__('rmf_api_server')


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    '''
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    '''
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


rclpy.init(args=None)
ros2_node = MainNode()
app = Flask(__name__)
if 'RMF_API_APP_ROOT' in os.environ:
    app.config['APPLICATION_ROOT'] = os.environ['RMF_API_ROOT']

threading.Thread(target=ros2_thread, args=[ros2_node]).start()
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

app.register_blueprint(make_blueprint(ros2_node))


@app.route('/building_map')
def get_building_map():
    return ros2_node.building_map

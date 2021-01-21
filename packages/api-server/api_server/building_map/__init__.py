import base64
import hashlib
import os
import threading
from typing import Optional

import flask
import rclpy
from building_map_msgs.msg import AffineImage, BuildingMap, Level
from flask import current_app
from rosidl_runtime_py.convert import message_to_ordereddict

from ..utils import urlpathjoin


class BuildingMapBlueprint(flask.Blueprint):
    def __init__(self, ros2_node):
        super().__init__('building_map', __name__, 'static')
        self.ros2_node = ros2_node
        self.building_map: Optional[BuildingMap] = None
        self.lock = threading.Lock()

    def make_setup_state(self, app, options, first_registration):
        state = super().make_setup_state(app, options, first_registration)
        if not first_registration:
            return state

        def _on_building_map(rmf_building_map: BuildingMap):
            '''
            1. Converts a `BuildingMap` message to an ordered dict.
            2. Saves the images into `static/cache/{map_name}`.
            3. Change the `AffineImage` `data` field to the url of the image.
            '''
            with self.lock:
                app.logger.info(
                    f'received new building map "{rmf_building_map.name}"')
                # this is inefficient as it converts a large bytearray to a list of numbers, but
                # there isn't any better alternatives
                self.building_map = message_to_ordereddict(
                    rmf_building_map)
                os.makedirs(
                    f'{self.static_folder}/cache/{rmf_building_map.name}', exist_ok=True)
                for i in range(len(rmf_building_map.levels)):
                    level: Level = rmf_building_map.levels[i]
                    for j in range(len(level.images)):
                        image: AffineImage = level.images[j]
                        # look at non-crypto hashes if we need more performance
                        sha1_hash = hashlib.sha1()
                        sha1_hash.update(image.data)
                        fingerprint = base64.b32encode(
                            sha1_hash.digest()).lower().decode()
                        relpath = f'cache/{rmf_building_map.name}/{level.name}-{image.name}.{fingerprint}.{image.encoding}'
                        filepath = f'{self.static_folder}/{relpath}'
                        # serve this with a more efficient webserver like nginx or a cdn if we need
                        # more performance
                        with open(filepath, 'bw') as f:
                            f.write(image.data)
                            app.logger.info(
                                f'saved map image to "{filepath}')
                        self.building_map['levels'][i]['images'][j]['data'] = \
                            urlpathjoin(options["url_prefix"],
                                        self.static_url_path, relpath)
                app.logger.info(
                    f'updated building map "{rmf_building_map.name}"')

        self.ros2_node.create_subscription(
            BuildingMap,
            'map',
            _on_building_map,
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

        return state


def building_map_blueprint(ros2_node):
    bp = BuildingMapBlueprint(ros2_node)

    @bp.route('/', methods=('GET',))
    def get_building_map():
        with bp.lock:
            if not bp.building_map:
                current_app.logger.error('no building map available')
                flask.abort(503)
            return bp.building_map

    return bp

# def make_building_map_blueprint(app, ros2_node):
#     bp = Blueprint('building_map', __name__, 'static')

#     def _on_building_map(rmf_building_map: BuildingMap):
#         '''
#         1. Converts a `BuildingMap` message to an ordered dict.
#         2. Saves the images into `static/cache/{map_name}`.
#         3. Change the `AffineImage` `data` field to the url of the image.
#         '''
#         app.logger.info(f'received new building map "{rmf_building_map.name}"')
#         # this is inefficient as it converts a large bytearray to a list of numbers, but
#         # there isn't any better alternatives
#         building_map = message_to_ordereddict(rmf_building_map)
#         os.makedirs(
#             f'{bp.static_folder}/cache/{rmf_building_map.name}', exist_ok=True)
#         for i in range(len(rmf_building_map.levels)):
#             level: Level = rmf_building_map.levels[i]
#             for j in range(len(level.images)):
#                 image: AffineImage = level.images[j]
#                 # look at non-crypto hashes if we need more performance
#                 sha1_hash = hashlib.sha1()
#                 sha1_hash.update(image.data)
#                 fingerprint = base64.b32encode(
#                     sha1_hash.digest()).lower().decode()
#                 urlpath = f'cache/{rmf_building_map.name}/{level.name}-{image.name}-{fingerprint}.{image.encoding}'
#                 filepath = f'{bp.static_folder}/{urlpath}'
#                 # serve this with a more efficient webserver like nginx or a cdn if we need
#                 # more performance
#                 with open(filepath, 'bw') as f:
#                     f.write(image.data)
#                     app.logger.info(f'saved map image to "{filepath}')
#                 building_map['levels'][i]['images'][j]['data'] = \
#                     urljoin(bp.static_url_path, urlpath)
#         app.logger.info(f'updated building map "{rmf_building_map.name}"')

#     ros2_node.create_subscription(
#         BuildingMap,
#         'map',
#         _on_building_map,
#         rclpy.qos.QoSProfile(
#             history=rclpy.qos.HistoryPolicy.KEEP_LAST,
#             depth=1,
#             reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
#             durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
#         )
#     )

#     @bp.route('/', methods=('GET',))
#     def get_building_map():
#         return building_map

#     return bp

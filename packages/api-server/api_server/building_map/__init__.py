from typing import Optional

from flask import Blueprint
from building_map_msgs.msg import AffineImage, BuildingMap, Level


def make_blueprint(ros2_node):
    bp = Blueprint('building_map', __name__, 'static')

    building_map: Optional[BuildingMap] = None

    def _on_building_map(building_map: BuildingMap):
        '''
        1. Converts a `BuildingMap` message to an ordered dict.
        2. Saves the images into `static/cache/{map_name}`.
        3. Change the `AffineImage` `data` field to the url of the image.
        '''
        app.logger.info(f'received new building map "{building_map.name}"')
        # this is inefficient as it converts a large bytearray to a list of numbers, but
        # there isn't any better alternatives
        self.building_map = message_to_ordereddict(building_map)
        os.makedirs(
            f'static/cache/{building_map.name}', exist_ok=True)
        for i in range(len(building_map.levels)):
            level: Level = building_map.levels[i]
            for j in range(len(level.images)):
                image: AffineImage = level.images[j]
                # look at non-crypto hashes if we need more performance
                sha1_hash = hashlib.sha1()
                sha1_hash.update(image.data)
                fingerprint = base64.b32encode(
                    sha1_hash.digest()).lower().decode()
                filepath = f'static/cache/{building_map.name}/{level.name}-{image.name}-{fingerprint}.{image.encoding}'
                # serve this with a more efficient webserver like nginx or a cdn if we need
                # more performance
                with open(filepath, 'bw') as f:
                    f.write(image.data)
                    app.logger.info(f'saved map image to "{filepath}"')
                self.building_map['levels'][i]['images'][j]['data'] = \
                    urljoin(app.config['APPLICATION_ROOT'], filepath)
        app.logger.info(f'updated building map "{building_map.name}"')

        building_map_sub = self.create_subscription(
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

    @bping_map.route('/')
    def get_building_map():
        return building_map

    return bp

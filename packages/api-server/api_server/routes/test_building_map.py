import rclpy.qos
from rmf_building_map_msgs.msg import BuildingMap as RmfBuildingMap

from .test_fixtures import RouteFixture


class TestBuildingMapRoute(RouteFixture):
    def test_get_building_map(self):
        rmf_building_map = RmfBuildingMap(name="test_map")
        pub = self.node.create_publisher(
            RmfBuildingMap,
            "map",
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_ALL,
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        pub.publish(rmf_building_map)
        resp = self.try_get(f"{self.base_url}/building_map")
        self.assertEqual(resp.status_code, 200)

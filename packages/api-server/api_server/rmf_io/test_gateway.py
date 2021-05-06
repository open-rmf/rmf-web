# import unittest

# from rmf_building_map_msgs.msg import AffineImage as RmfAffineImage
# from rmf_building_map_msgs.msg import BuildingMap as RmfBuildingMap
# from rmf_building_map_msgs.msg import Level as RmfLevel

# from .gateway import process_building_map


# class TestProcessBuildingMap(unittest.TestCase):
#     def test_convert_building_map(self):
#         rmf_building_map = RmfBuildingMap(
#             name="test_map", levels=[RmfLevel(images=[RmfAffineImage(data=b"test")])]
#         )
#         static_files = unittest.mock.Mock()
#         static_files.add_file.return_value = "test_url"
#         building_map = process_building_map(rmf_building_map, static_files)
#         self.assertEqual(building_map.levels[0].images[0].data, "test_url")

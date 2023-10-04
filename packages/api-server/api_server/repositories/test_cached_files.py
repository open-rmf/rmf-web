import os.path
import unittest
from os.path import dirname

from .cached_files import CachedFilesRepository


class TestCachedFilesRepository(unittest.TestCase):
    def setUp(self):
        self.artifact_dir = f"{dirname(dirname(dirname(__file__)))}/test_artifacts"
        self.repo = CachedFilesRepository("/cache", self.artifact_dir)

    def test_add_file(self):
        """
        test file is saved in the correct location and it returns the correct url
        """
        target_path = "TestCachedFilesRepository/test_add_file.txt"
        url_path = self.repo.add_file(b"hello", target_path)
        self.assertEqual(url_path, f"/cache/{target_path}")
        saved_path = f"{self.artifact_dir}/{target_path}"
        self.assertTrue(os.path.exists(saved_path))
        with open(saved_path, "br") as f:
            self.assertEqual(b"hello", f.read())

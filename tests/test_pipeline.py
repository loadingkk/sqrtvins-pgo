import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location("pipeline", Path(__file__).resolve().parents[1] / "scripts/pipeline.py")
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


class PacketInputTest(unittest.TestCase):
    def test_contiguous_pairs_and_missing_file(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for index in (0, 1):
                for extension in ("txt", "png"):
                    (root / f"{index:06}.{extension}").touch()
            self.assertEqual(pipeline.packet_count(root), 2)
            (root / "000001.png").unlink()
            with self.assertRaisesRegex(ValueError, "000001.png"):
                pipeline.packet_count(root)

    def test_rejects_dropped_export(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "summary.json").write_text(json.dumps({"written": 2, "dropped": 1, "failed": 0}))
            with self.assertRaisesRegex(ValueError, "export was incomplete"):
                pipeline.packet_count(root)

    def test_opencv_yaml_header(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / "config.yaml"
            config.write_text("%YAML:1.0\n---\ncalib_cam_extrinsics: false\n")
            self.assertEqual(pipeline.read_config(config), {"calib_cam_extrinsics": False})

    def test_check_uses_the_configured_topics_and_camera_models(self):
        config = pipeline.PROJECT / "examples/euroc/estimator_config.yaml"
        topics = {
            "/imu0": SimpleNamespace(msg_type="sensor_msgs/Imu", message_count=200),
            "/cam0/image_raw": SimpleNamespace(msg_type="sensor_msgs/Image", message_count=20),
            "/cam1/image_raw": SimpleNamespace(msg_type="sensor_msgs/Image", message_count=20),
        }
        with patch.dict(sys.modules, {"rosbag": SimpleNamespace()}) as modules:
            with patch.object(modules["rosbag"], "Bag", create=True) as bag:
                bag.return_value.__enter__.return_value.get_type_and_topic_info.return_value = SimpleNamespace(topics=topics)
                info = pipeline.inspect_inputs(Path("example.bag"), config, 2)
                self.assertEqual(info["camera_models"], {"cam0": "pinhole-radtan", "cam1": "pinhole-radtan"})
                self.assertEqual(info["topics"]["/imu0"]["messages"], 200)
                topics["/cam1/image_raw"].message_count = 0
                with self.assertRaisesRegex(ValueError, "/cam1/image_raw"):
                    pipeline.inspect_inputs(Path("example.bag"), config, 2)


if __name__ == "__main__":
    unittest.main()

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "bag_preflight.py"
SPEC = importlib.util.spec_from_file_location("bag_preflight", SCRIPT)
bag_preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bag_preflight)


class BagPreflightTest(unittest.TestCase):
    def setUp(self):
        self.contract = {
            "profiles": {
                "rtk_fusion": {
                    "/cloud_registered_body": "sensor_msgs/msg/PointCloud2",
                    "/Odometry": "nav_msgs/msg/Odometry",
                    "/odometry/gps": "nav_msgs/msg/Odometry",
                    "/rtk/heading_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
                }
            }
        }

    def metadata(self):
        topics = []
        for name, message_type in self.contract["profiles"]["rtk_fusion"].items():
            topics.append(
                {
                    "topic_metadata": {"name": name, "type": message_type},
                    "message_count": 100,
                }
            )
        return {
            "rosbag2_bagfile_information": {
                "duration": {"nanoseconds": 10_000_000_000},
                "topics_with_message_count": topics,
            }
        }

    def test_valid_contract_reports_rates(self):
        report = bag_preflight.evaluate(self.metadata(), self.contract, "rtk_fusion")
        self.assertTrue(report["ok"])
        self.assertEqual(report["duration_seconds"], 10.0)
        self.assertEqual(report["present"]["/Odometry"]["average_hz"], 10.0)

    def test_missing_empty_and_wrong_type_fail(self):
        metadata = self.metadata()
        topics = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
        topics[:] = [topic for topic in topics if topic["topic_metadata"]["name"] != "/odometry/gps"]
        topics[0]["message_count"] = 0
        topics[1]["topic_metadata"]["type"] = "geometry_msgs/msg/PoseStamped"

        report = bag_preflight.evaluate(metadata, self.contract, "rtk_fusion")

        self.assertFalse(report["ok"])
        self.assertEqual(report["missing"], ["/odometry/gps"])
        self.assertEqual(report["empty"], ["/cloud_registered_body"])
        self.assertEqual(report["type_mismatches"][0]["topic"], "/Odometry")


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import os
import shlex
import signal
import shutil
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class BagRecorder(Node):
    def __init__(self):
        super().__init__("bag_recorder")

        self.declare_parameter("bag_dir", os.path.expanduser("~/teleop_bags"))
        self.declare_parameter("topics", [
            "/robot/front_rgbd_camera/color/camera_info",
            "/robot/front_rgbd_camera/color/image_raw/compressed",
            "/Odometry",
            "/cmd_vel",
            "/cmd_vel_web",
            "/dwal_planner/cluster_markers_far",
            "/dwal_planner/cluster_markers_near",
            "/dwal_planner/clusters_far",
            "/dwal_planner/clusters_near",
            "/dwal_planner/sampled_pathMarkers",
            "/dwal_planner/sampled_paths",
            "/joy_web",
            "/local_costmap/costmap",
            "/local_costmap/costmap_raw",
            "/local_costmap/costmap_raw_updates",
            "/local_costmap/costmap_updates",
            "/local_costmap/footprint",
            "/local_costmap/local_costmap/transition_event",
            "/local_costmap/obstacle_layer",
            "/local_costmap/obstacle_layer_raw",
            "/local_costmap/obstacle_layer_raw_updates",
            "/local_costmap/obstacle_layer_updates",
            "/local_costmap/published_footprint",
            "/robot/shared_control_teleop/cmd_vel",
            "/shared_controller/intent_plot",
            "/shared_controller/path_human",
            "/shared_controller/path_shared",
            "/tf",
            "/tf_static"
        ])

        self.bag_dir = self.get_parameter("bag_dir").get_parameter_value().string_value
        self.topics = list(self.get_parameter("topics").get_parameter_value().string_array_value)

        os.makedirs(self.bag_dir, exist_ok=True)

        self.proc = None
        self.current_bag = ""
        self.last_finished_bag = ""

        self.create_service(Trigger, "bag_record_start", self.start_cb)
        self.create_service(Trigger, "bag_record_stop", self.stop_cb)
        self.create_service(Trigger, "bag_record_delete_last", self.delete_last_cb)

        self.get_logger().info(f"Bag recorder ready. Output dir: {self.bag_dir}")

    def _make_bag_name(self) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.bag_dir, f"teleop_bag_{stamp}")

    def start_cb(self, request, response):
        if self.proc is not None:
            response.success = False
            response.message = f"Already recording: {self.current_bag}"
            return response

        self.current_bag = self._make_bag_name()

        cmd = ["ros2", "bag", "record", "-o", self.current_bag] + self.topics
        self.get_logger().info("Starting rosbag: " + " ".join(shlex.quote(x) for x in cmd))

        try:
            self.proc = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid,  # own process group
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.proc = None
            response.success = False
            response.message = f"Failed to start recording: {e}"
            return response

        response.success = True
        response.message = self.current_bag
        return response

    def stop_cb(self, request, response):
        if self.proc is None:
            response.success = False
            response.message = "Not recording"
            return response

        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("rosbag did not exit on SIGINT, killing")
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            self.proc.wait(timeout=5)
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop recording: {e}"
            return response

        self.last_finished_bag = self.current_bag
        finished = self.last_finished_bag
        self.proc = None
        self.current_bag = ""

        response.success = True
        response.message = finished
        return response

    def delete_last_cb(self, request, response):
        if self.proc is not None:
            response.success = False
            response.message = "Stop recording before deleting"
            return response

        if not self.last_finished_bag:
            response.success = False
            response.message = "No finished bag to delete"
            return response

        if not os.path.isdir(self.last_finished_bag):
            deleted = self.last_finished_bag
            self.last_finished_bag = ""
            response.success = False
            response.message = f"Bag folder not found: {deleted}"
            return response

        try:
            shutil.rmtree(self.last_finished_bag)
            deleted = self.last_finished_bag
            self.last_finished_bag = ""
            response.success = True
            response.message = deleted
        except Exception as e:
            response.success = False
            response.message = f"Delete failed: {e}"

        return response


def main():
    rclpy.init()
    node = BagRecorder()
    try:
        rclpy.spin(node)
    finally:
        if node.proc is not None:
            try:
                os.killpg(os.getpgid(node.proc.pid), signal.SIGINT)
                node.proc.wait(timeout=5)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

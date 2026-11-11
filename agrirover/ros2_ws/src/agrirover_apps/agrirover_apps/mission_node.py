"""Inspection mission executor on Nav2.

Sequence per mission: dock -> for each station (serpentine order): NavigateToPose -> settle -> grab one frame per
camera -> vision pipeline -> write observations + environment sample to SQLite -> next ... -> dock.
The dashboard reads the same database and calls ~/start, ~/stop (std_srvs/Trigger).

This node reuses the hardware-independent modules that are tested in software/tests (route ordering, vision pipeline,
storage). The sequencing here mirrors agrirover.runtime.Rover but delegates motion to Nav2.
STATUS: not yet executed against ROS 2 (no ROS runtime in the authoring environment). Known gaps before field use:
  * per-camera rectification (homography from the tilted view to the bench plane) must be calibrated on site and its
    file passed in `rectification_file`; without it frames are stored but plant metrics are marked "unreliable".
  * battery return threshold uses /battery_state.percentage (coulomb counting from the base node).
"""
import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agrirover.storage.db import Store
from agrirover.vision import pipeline, quality


def pose_msg(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = "map", nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class MissionNode(Node):
    def __init__(self):
        super().__init__("agrirover_mission")
        self.declare_parameter("stations_file", "")
        self.declare_parameter("database", str(Path.home() / "agrirover_data" / "agrirover.db"))
        self.declare_parameter("rectification_file", "")
        self.declare_parameter("settle_time", 1.5)
        self.declare_parameter("soc_return", 0.25)
        cfg = yaml.safe_load(Path(self.get_parameter("stations_file").value).read_text())
        self.dock, self.stations, self.plants = cfg["dock"], cfg["stations"], cfg["plants"]
        self.rows, self.cols = cfg["rows"], cfg["plants_per_row"]
        self.store = Store(self.get_parameter("database").value)
        rf = self.get_parameter("rectification_file").value
        self.rect = yaml.safe_load(Path(rf).read_text()) if rf else None
        self.bridge = CvBridge()
        self.frames, self.env, self.soc = {}, None, 1.0
        self.create_subscription(Image, "camera_left/image_raw", lambda m: self.frames.__setitem__("left", m), 2)
        self.create_subscription(Image, "camera_right/image_raw", lambda m: self.frames.__setitem__("right", m), 2)
        self.create_subscription(String, "env/json", lambda m: setattr(self, "env", json.loads(m.data)), 5)
        self.create_subscription(BatteryState, "battery_state", lambda m: setattr(self, "soc", m.percentage), 5)
        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.nav = BasicNavigator()
        self.thread = None
        self.stop_flag = threading.Event()

    def on_start(self, _req, res):
        if self.thread and self.thread.is_alive():
            res.success, res.message = False, "mission already running"
        else:
            self.stop_flag.clear()
            self.thread = threading.Thread(target=self.run_mission, daemon=True)
            self.thread.start()
            res.success, res.message = True, "started"
        return res

    def on_stop(self, _req, res):
        self.stop_flag.set()
        self.nav.cancelTask()
        res.success, res.message = True, "stopping, returning to dock"
        return res

    def goto(self, x, y, yaw) -> bool:
        self.nav.goToPose(pose_msg(self.nav, x, y, yaw))
        while not self.nav.isTaskComplete():
            if self.stop_flag.is_set():
                self.nav.cancelTask()
                return False
            time.sleep(0.1)
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def order(self):
        out = []
        for n, a in enumerate(sorted({s["aisle"] for s in self.stations})):
            st = sorted([s for s in self.stations if s["aisle"] == a], key=lambda s: s["x"])
            out += [(s, 0.0) for s in st] if n % 2 == 0 else [(s, math.pi) for s in st[::-1]]
        return out

    def capture(self, mid, station, heading, env_id):
        flipped = abs(heading) > math.pi / 2
        for view in station["views"]:
            cam = view["camera"] if not flipped else ("right" if view["camera"] == "left" else "left")
            msg = self.frames.get(cam)
            now = time.time()
            if msg is None:
                self.store.add_event("warn", "CAMERA_NO_FRAME", f"{station['id']}/{cam}", mid)
                continue
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            name = f"m{mid:04d}_{station['id']}_{cam}.jpg"
            cv2.imwrite(str(self.store.image_dir / name), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            pids = list(view["plants"])
            if flipped:
                pids = [p for r in range(self.rows) for p in reversed(pids[r * self.cols:(r + 1) * self.cols])]
            if self.rect and cam in self.rect:
                c = self.rect[cam]
                rectified = cv2.warpPerspective(img, np.array(c["homography"], float), tuple(c["size"]))
                slots = [(pid, *c["slots"][k]) for k, pid in enumerate(pids)]
                res = pipeline.analyse(rectified, slots, c["px_per_m"], tuple(c["grey_patch"]) if c.get("grey_patch") else None)
                obs, q, level = res["plants"], res["quality"], res["level"]
            else:                                   # no calibration: keep the image, make no plant claims
                q = quality.assess(img)
                obs, level = [dict(plant_id=pid, status="unreliable", flags={}) for pid in pids], "UNCALIBRATED"
            for o in obs:
                self.store.add_observation(mid, now, self.plants[o["plant_id"]], station["id"], cam, (station["x"], station["y"], heading),
                                           name, o, q, env_id, level)

    def run_mission(self):
        order = self.order()
        mid = self.store.start_mission("inspection", len(order))
        self.store.add_event("info", "MISSION_START", f"{len(order)} stations", mid)
        self.nav.waitUntilNav2Active()
        done, status = 0, "done"
        for station, heading in order:
            if self.stop_flag.is_set():
                status = "aborted"
                break
            if self.soc < self.get_parameter("soc_return").value:
                self.store.add_event("warn", "BATTERY_LOW", f"SoC {self.soc*100:.0f} %", mid)
                status = "aborted"
                break
            if not self.goto(station["x"], station["y"], heading):
                self.store.add_event("warn", "STATION_SKIPPED", station["id"], mid)
                continue
            time.sleep(self.get_parameter("settle_time").value)
            env_id = self.store.add_env(mid, time.time(), station["x"], station["y"], station["id"], self.env) if self.env else None
            self.capture(mid, station, heading, env_id)
            done += 1
            self.store.update_mission(mid, stations_done=done)
        self.stop_flag.clear()
        ok = self.goto(self.dock["x"], self.dock["y"], self.dock["yaw"])
        self.store.update_mission(mid, finished=time.time(), status=status if ok else "aborted", stations_done=done, summary=dict(docked=ok))
        self.store.add_event("info", "MISSION_DONE" if ok and status == "done" else "MISSION_ABORTED", status, mid)


def main():
    rclpy.init()
    node = MissionNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

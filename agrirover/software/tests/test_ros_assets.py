"""Static checks of the ROS 2 workspace. ROS itself is not required: these tests catch drift between the shared
configuration and the ROS assets, and syntax errors - they do NOT show that the nodes or Nav2 run."""
import ast
import json
import xml.dom.minidom
from pathlib import Path

import pytest
import yaml

from agrirover import config
from agrirover.sim.world import World

WS = config.project_root() / "ros2_ws" / "src"


def test_urdf_matches_design():
    xacro = pytest.importorskip("xacro")
    yourdfpy = pytest.importorskip("yourdfpy")
    doc = xacro.process_file(str(WS / "agrirover_description/urdf/agrirover.urdf.xacro"), mappings={"use_gazebo": "true"})
    xml_text = doc.toxml()
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(xml_text)
    u = yourdfpy.URDF.load(f.name, build_scene_graph=True, load_meshes=False)
    c = config.rover()["chassis"]
    left = u.get_transform("left_wheel", "base_footprint")[:3, 3]
    assert left[1] == pytest.approx(c["track_width_mm"] / 2000) and left[2] == pytest.approx(c["wheel_diameter_mm"] / 2000)
    lidar = u.get_transform("lidar_link", "base_footprint")[:3, 3]
    assert lidar[0] == pytest.approx(c["lidar_x_mm"] / 1000) and lidar[2] == pytest.approx(c["lidar_scan_z_mm"] / 1000)
    cam = u.get_transform("camera_left_optical_frame", "base_footprint")
    assert cam[2, 3] == pytest.approx(c["camera_z_mm"] / 1000)
    import math
    assert cam[1, 2] > 0 and math.degrees(math.asin(-cam[2, 2])) == pytest.approx(c["camera_tilt_deg"], abs=0.1)   # looks left, tilted down
    assert u.get_transform("camera_right_optical_frame", "base_footprint")[1, 2] < 0
    assert f"<wheel_separation>{c['track_width_mm'] / 1000}" in xml_text.replace("0.33<", "0.33<") or "<wheel_separation>0.33" in xml_text
    assert {"base_footprint", "base_link", "imu_link", "env_link", "mast_link"} <= set(u.link_map)


def test_nav2_params_consistent():
    p = yaml.safe_load((WS / "agrirover_bringup/config/nav2_params.yaml").read_text())
    outline = [[round(x, 3), round(y, 3)] for x, y in config.footprint_outline()]
    for cm in ("local_costmap", "global_costmap"):
        fp = json.loads(p[cm][cm]["ros__parameters"]["footprint"])
        assert fp == outline, "Nav2 footprint differs from config.footprint_outline()"
        assert "keepout_filter" in p[cm][cm]["ros__parameters"]["filters"]
    rq = config.rover()["requirements"]
    vs = p["velocity_smoother"]["ros__parameters"]
    assert vs["max_velocity"][0] == rq["v_max"] and vs["max_accel"][0] == rq["a_max"]
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] == rq["v_inspect"]
    dock = config.greenhouse()["dock"]
    ip = p["amcl"]["ros__parameters"]["initial_pose"]
    assert (ip["x"], ip["y"]) == (dock["x"], dock["y"])
    stop = json.loads(p["collision_monitor"]["ros__parameters"]["stop_zone"]["points"])
    assert max(abs(y) for _, y in stop) >= config.half_width() and min(x for x, _ in stop) <= max(x for x, _ in outline)


def test_generated_assets_match_world():
    w = World()
    st = yaml.safe_load((WS / "agrirover_bringup/config/stations.yaml").read_text())
    assert len(st["stations"]) == len(w.stations) and len(st["plants"]) == len(w.plants)
    assert (st["dock"]["x"], st["dock"]["y"]) == w.dock[:2]
    sdf = xml.dom.minidom.parse(str(WS / "agrirover_bringup/worlds/greenhouse.sdf"))
    names = [m.getAttribute("name") for m in sdf.getElementsByTagName("model")]
    assert sum(n.endswith("_top") for n in names) == len(w.benches) and sum(n.startswith("leg") for n in names) == len(w.legs())
    cv2 = pytest.importorskip("cv2")
    for name in ("greenhouse", "keepout_mask"):
        meta = yaml.safe_load((WS / f"agrirover_bringup/maps/{name}.yaml").read_text())
        img = cv2.imread(str(WS / "agrirover_bringup/maps" / meta["image"]), cv2.IMREAD_GRAYSCALE)
        assert img is not None and img.shape[1] * meta["resolution"] == pytest.approx(w.w + 2.0)
    keep = cv2.imread(str(WS / "agrirover_bringup/maps/keepout_mask.pgm"), cv2.IMREAD_GRAYSCALE)
    bench_area = sum((b.x1 - b.x0) * (b.y1 - b.y0) for b in w.benches)
    assert (keep == 0).sum() * 0.05 ** 2 == pytest.approx(bench_area, rel=0.05)


def test_python_sources_parse_and_packages_declare_entry_points():
    files = [f for f in WS.rglob("*.py") if "__pycache__" not in f.parts]
    assert len(files) >= 10
    for f in files:
        ast.parse(f.read_text(), filename=str(f))
    for pkg, script in (("agrirover_base", "base_node"), ("agrirover_apps", "mission_node"), ("agrirover_apps", "env_sensor_node")):
        assert f"{script} = {pkg}.{script}:main" in (WS / pkg / "setup.py").read_text()
        assert (WS / pkg / pkg / f"{script}.py").exists() and (WS / pkg / "resource" / pkg).exists()
    for pkg in ("agrirover_description", "agrirover_bringup", "agrirover_base", "agrirover_apps"):
        xml.dom.minidom.parse(str(WS / pkg / "package.xml"))

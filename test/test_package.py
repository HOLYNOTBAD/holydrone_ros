#!/usr/bin/env python3

import pathlib
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]


class HolydronePackageTest(unittest.TestCase):
    def test_xml_files_parse(self):
        files = [ROOT / "package.xml"]
        files += list((ROOT / "gazebo").rglob("*.sdf"))
        files += list((ROOT / "gazebo").rglob("model.config"))
        files += list((ROOT / "gazebo/worlds").glob("*.world"))
        files += list((ROOT / "launch").glob("*.launch"))
        for path in files:
            with self.subTest(path=path.relative_to(ROOT)):
                ET.parse(path)

    def test_model_uri_dependencies_are_bundled(self):
        descriptions = [ROOT / "gazebo/models/holydrone/holydrone.sdf"]
        descriptions += list((ROOT / "gazebo/worlds").glob("*.world"))
        model_names = []
        for description in descriptions:
            model_names += re.findall(
                r"model://([^/<]+)", description.read_text()
            )
        for name in model_names:
            with self.subTest(model=name):
                if name != "sun":
                    self.assertTrue((ROOT / "gazebo/models" / name).is_dir())

    def test_required_assets_exist(self):
        required = [
            "gazebo/models/holydrone/meshes/holydrone_v1.STL",
            "gazebo/models/holydrone/meshes/Propeller 7inch.STL",
            "gazebo/models/lidar/model.sdf",
            "gazebo/worlds/baylands.world",
            "gazebo/models/holydrone_baylands/baylands.sdf",
            "gazebo/models/holydrone_baylands/media/baylands_01.DAE",
            "gazebo/models/holydrone_baylands/media/baylands_02.DAE",
            "gazebo/models/holydrone_baylands/media/baylands_03.DAE",
            "gazebo/models/holydrone_baylands/media/Textures/Grass.png",
            "px4/overlay/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gazebo-classic_holydrone",
        ]
        for relative in required:
            with self.subTest(path=relative):
                self.assertTrue((ROOT / relative).is_file())

    def test_physical_camera_calibration_is_not_bundled(self):
        self.assertFalse((ROOT / "camera").exists())

    def test_interception_controller_profiles_are_not_bundled(self):
        self.assertFalse((ROOT / "control_interception").exists())

    def test_model_only_spawn_entry_is_not_bundled(self):
        self.assertFalse((ROOT / "launch/spawn.launch").exists())
        self.assertFalse((ROOT / "scripts/check_px4_plugins.sh").exists())

    def test_sitl_is_declared_in_launch(self):
        launch = (ROOT / "launch/sitl.launch").read_text()
        self.assertFalse((ROOT / "scripts/run_sitl.sh").exists())
        self.assertIn('/home/holy/bvr/third_party/PX4-Autopilot', launch)
        self.assertIn('pkg="gazebo_ros" type="spawn_model"', launch)
        self.assertIn('name="px4_sitl"', launch)
        self.assertNotIn("auto_takeoff", launch)
        self.assertNotIn("HOLYDRONE_PX4_DIR", launch)

    def test_px4_runs_without_interactive_stdin(self):
        launch = ET.parse(ROOT / "launch/sitl.launch").getroot()
        node = next(node for node in launch.findall("node")
                    if node.attrib.get("name") == "px4_sitl")
        self.assertIn('bin/px4" -d ', node.attrib["launch-prefix"])

    def test_sim_mavros_is_owned_by_the_sim_package(self):
        launch = (ROOT / "launch/mavros_sim.launch").read_text()
        self.assertIn("$(find mavros)/launch/px4.launch", launch)
        self.assertIn("configure_mavlink_streams.py", launch)
        self.assertTrue((ROOT / "scripts/configure_mavlink_streams.py").is_file())

    def test_removed_deployment_shell_helpers_are_not_bundled(self):
        for name in ["check_install.sh", "install_px4_overlay.sh", "setup_env.sh"]:
            with self.subTest(name=name):
                self.assertFalse((ROOT / "scripts" / name).exists())

    def test_runtime_declares_ros_gazebo_camera_plugin(self):
        package = ET.parse(ROOT / "package.xml").getroot()
        dependencies = {node.text for node in package.findall("exec_depend")}
        self.assertIn("gazebo_plugins", dependencies)


if __name__ == "__main__":
    unittest.main()

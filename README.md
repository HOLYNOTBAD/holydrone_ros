# holydrone

`holydrone` is the ROS 1 aircraft integration package for simulation and real
flight. It contains the portable Gazebo Classic model, PX4 SITL airframe
overlay, MAVROS launch entries, camera helpers and visualization nodes.

## Package contents

| Path | Purpose |
| --- | --- |
| `gazebo/models/holydrone` | Airframe SDF and Git LFS-managed body/propeller meshes |
| `gazebo/models/{gps,lidar,fpv_cam}` | Sensor submodels required by the airframe SDF |
| `gazebo/models/tpv_cam` | Optional third-person Gazebo camera |
| `gazebo/worlds/baylands.world` | Portable Baylands scene with a flat takeoff pad |
| `gazebo/models/holydrone_baylands` | Baylands Collada meshes, textures and material assets managed by Git LFS |
| `px4/overlay` | Backed-up PX4 `22000` airframe using the original PX4 source-tree layout |
| `scripts` | Camera synchronization and visualization helpers |

The supported baseline is recorded in [`px4/VERSION.md`](px4/VERSION.md).
The package does not redistribute compiled PX4/Gazebo plugins because they are
ABI-dependent; they are built from the pinned PX4 source instead.

## Add to a Catkin workspace

Clone the package into a ROS Noetic workspace and fetch its Git LFS assets:

```bash
cd /path/to/catkin_ws/src
git clone https://github.com/HOLYNOTBAD/holydrone_ros.git holydrone
git -C holydrone lfs pull

cd ..
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Projects that need to pin an exact tested revision should add this repository
as a Git submodule at `src/holydrone` instead of copying the package:

```bash
git submodule add \
  https://github.com/HOLYNOTBAD/holydrone_ros.git \
  src/holydrone
git submodule update --init --recursive
git -C src/holydrone lfs pull
```

## Utilities

Synchronize the bundled SDF camera pose and intrinsics from a calibration:

```bash
rosrun holydrone sync_gazebo_camera_model.py \
  --config "$(rospack find camera_driver)/config/camera/camera_params_usb_imx577_30degree.yaml"
```

Camera calibration YAML files intentionally live in the host project's
`camera_driver/config/camera` directory. They describe a particular physical camera and are
not part of the portable aircraft simulation package.

ISB-NMPC interception profiles likewise remain in the host project's
`isb_nmpc/config/holydrone` directory because they describe
controller behavior rather than the aircraft simulation model.

After the complete SITL entry is running, the optional third-person camera
can be started separately:

```bash
roslaunch holydrone tpv_camera.launch
```

## PX4 deployment and launch

The validated baseline is PX4-Autopilot `v1.16.2`, ROS Noetic and Gazebo
Classic 11. Clone PX4 with submodules and install its Ubuntu dependencies:

```bash
cd ~
git clone --recursive --branch v1.16.2 \
  https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
```

### 1. Copy the Holydrone airframe into PX4

Open a new terminal and point `PX4_DIR` at the PX4 tree that will be used by
`sitl.launch`:

```bash
cd /path/to/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PX4_DIR=~/PX4-Autopilot
export HOLYDRONE_DIR="$(rospack find holydrone)"

install -m 0755 \
  "$HOLYDRONE_DIR/px4/overlay/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gazebo-classic_holydrone" \
  "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/22000_gazebo-classic_holydrone"
```

### 2. Register airframe 22000

Edit this PX4 file:

```text
$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

Inside its `px4_add_romfs_files(...)` list, add exactly one entry:

```cmake
        22000_gazebo-classic_holydrone
```

For PX4 trees previously configured for this project, replace the legacy
`22000_gazebo-classic_holydrone_cam` entry instead of keeping both names. The
following commands perform that registration using the existing
`71001_gazebo-classic_spacecraft_dart` entry as the insertion anchor:

```bash
PX4_AIRFRAME_CMAKE="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"

if grep -q '22000_gazebo-classic_holydrone_cam' "$PX4_AIRFRAME_CMAKE"; then
  sed -i 's/22000_gazebo-classic_holydrone_cam/22000_gazebo-classic_holydrone/' \
    "$PX4_AIRFRAME_CMAKE"
elif ! grep -q '22000_gazebo-classic_holydrone' "$PX4_AIRFRAME_CMAKE"; then
  sed -i '/71001_gazebo-classic_spacecraft_dart/i\        22000_gazebo-classic_holydrone' \
    "$PX4_AIRFRAME_CMAKE"
fi
```

Verify that the registration appears exactly once:

```bash
grep -n '22000_gazebo-classic_holydrone' "$PX4_AIRFRAME_CMAKE"
```

### 3. Initialize submodules and compile PX4

Run both build commands after copying or changing the airframe:

```bash
git -C "$PX4_DIR" submodule update --init --recursive
make -C "$PX4_DIR" px4_sitl_default
DONT_RUN=1 make -C "$PX4_DIR" px4_sitl gazebo-classic
```

The first command generates the PX4 SITL binary and copies airframe 22000 into
the build tree. The second command builds the Gazebo Classic plugins without
starting a simulation.

### 4. Verify the build products

```bash
export PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

test -x "$PX4_BUILD_DIR/bin/px4"
test -f "$PX4_BUILD_DIR/etc/init.d-posix/airframes/22000_gazebo-classic_holydrone"

for plugin in \
  libgazebo_barometer_plugin.so \
  libgazebo_gps_plugin.so \
  libgazebo_groundtruth_plugin.so \
  libgazebo_imu_plugin.so \
  libgazebo_lidar_plugin.so \
  libgazebo_magnetometer_plugin.so \
  libgazebo_mavlink_interface.so \
  libgazebo_motor_model.so \
  libgazebo_multirotor_base_plugin.so; do
  test -f "$PX4_BUILD_DIR/build_gazebo-classic/$plugin" || {
    echo "Missing PX4 Gazebo plugin: $plugin" >&2
    exit 1
  }
done

echo "PX4 SITL, airframe 22000 and Gazebo Classic plugins are ready."
```

If the PX4 build still contains the old `_cam` filename, cleanly rebuild
`px4_sitl_default` after updating the source-tree `CMakeLists.txt`.

### 5. Start the simulation

The package intentionally exposes one aircraft simulation entry point:
`sitl.launch`. The launch file directly configures Gazebo paths, starts the
world, inserts Holydrone, starts PX4 SITL and finally opens the GUI. Set the
`px4_dir` default near the top of `launch/sitl.launch` for the target computer:

```bash
roslaunch holydrone sitl.launch
```

The interception launcher starts MAVROS separately through the ROS package's
second infrastructure entry. It waits for the FCU connection before setting
the simulated IMU message intervals:

```bash
roslaunch holydrone mavros_sim.launch
```

To use another PX4 tree without editing the file, override the launch argument:

```bash
roslaunch holydrone sitl.launch \
  px4_dir:=/path/to/PX4-Autopilot
```

The default is the bundled Baylands scene. Its takeoff-pad top is at
`z=0.50 m`, so `sitl.launch` uses `spawn_z=0.555 m` after adding the airframe
clearance. The default command is therefore simply:

```bash
roslaunch holydrone sitl.launch
```

For a lighter empty scene, select both the world and its ground-level spawn
height:

```bash
roslaunch holydrone sitl.launch \
  world:="$(rospack find holydrone)/gazebo/worlds/empty.world" \
  spawn_z:=0.055
```

The Baylands world is self-contained in this ROS package. It uses the unique
`model://holydrone_baylands` URI, so it does not depend on or get shadowed by
`~/.gazebo/models/baylands` or the host project's `params/gazebo` directory.
Because its meshes and textures are stored with Git LFS, run `git lfs pull`
after cloning a repository that contains this package.

The launch file does not check the MAVLink connection or send arm/takeoff
commands. The vehicle remains on the ground for QGroundControl or MAVROS
operation. Set `reset_params:=true` only when persisted PX4 SITL parameters
should be removed. Press `Ctrl-C` to stop PX4 and Gazebo.

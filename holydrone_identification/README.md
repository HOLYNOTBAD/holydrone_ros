# holydrone_identification

最短使用流程：构建后，实机用 `record.launch` 被动采集；Gazebo 用 `sim.launch`
采集激励数据。分别采集训练和验证两次，运行 `identify.py response` 得到一阶模型。
实机物理参数使用 `identify.py physics`，必须提供测量过的质量、几何和电机顺序。
默认不自动修改飞机参数、不启动 RL 训练。测试结果见 [VALIDATION.md](VALIDATION.md)。

ROS Noetic package for two different identification layers:

1. **Real aircraft → Gazebo physical candidates**: passive flight recording, known-mass
   rotor thrust coefficient, aggregate diagonal inertia, rotor reaction torque and angular damping;
   a separate bench CSV workflow estimates motor thrust/torque coefficients and speed dynamics.
2. **Gazebo/PX4 → training response model**: bounded single-axis hover pulses, first-order
   gain/time constant/delay/bias fitting, independent validation, and optional QUAD_RL configuration export.

The ROS name uses underscores (`holydrone_identification`). This directory is a child of the
`holydrone_ros` repository, but a separate catkin package. **Catkin does not recursively discover
packages inside another package**. Expose it as a separate workspace source link:

```bash
cd /path/to/catkin_ws/src
# holydrone is the clone of holydrone_ros
ln -s holydrone/holydrone_identification holydrone_identification
cd ..
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack find holydrone_identification
```

QUAD_RL's `deployment/_scripts/build_ros.sh` creates this link automatically.
Dependencies: rospy, MAVROS messages, NumPy, SciPy, PyYAML. No JAX dependency.

## 1. Real flight data collection

Read-only recording with an existing MAVROS connection:

```bash
roslaunch holydrone_identification record.launch \
  environment:=real output:=/absolute/path/flight_001
```

Or start the aircraft package's real MAVROS entry and recorder together:

```bash
export MAVROS_FCU_URL=/dev/ttyTHS0:2000000
export MAVROS_TGT_SYSTEM=2
roslaunch holydrone_identification real.launch output:=/absolute/path/flight_001
```

The recorder neither arms nor sends flight commands. Use a new output directory each time.
Stop with Ctrl+C after the flight. Outputs are `samples.csv` and `metadata.json`. Also preserve
PX4 ULog and the complete PX4 parameter export for the same flight: ROS configuration alone
cannot reproduce the autopilot configuration.

Rows use IMU header timestamps and select the latest command whose timestamp is not later
than that IMU sample. The CSV retains `cmd_t`, experiment channel/segment/phase, position,
velocity, attitude, body rate, specific force, battery voltage, armed/mode and four ESC RPMs.
Duplicate/out-of-order IMU timestamps are dropped. `valid` is for active response pulses;
`flight_valid` and `rpm_valid` are separate physics-data checks. Missing RPM remains NaN;
normalized collective thrust is never substituted for motor RPM.

Conventions: position/velocity ENU; rates and IMU specific force in body FLU; a level stationary
calibrated accelerometer reports approximately +9.81 m/s² along z. Mounting alignment, gyro and
accelerometer biases, lever-arm effects and time synchronization must be calibrated before
physics fitting. RPM must be **mechanical RPM**, not unconverted electrical RPM. Confirm ESC
telemetry availability, indexing and update rate on the actual firmware; the recorder uses
`/mavros/esc_status`. For other telemetry sources, convert the data to the documented CSV units.

### Physical parameters from flight logs

Copy `config/survey.example.yaml` and enter the measured mass, center of mass, rotor positions,
body reaction-torque signs, IMU bias and verified ESC ordering. The example intentionally has
zero mass and `motor_order_verified: false` so it cannot be mistaken for calibrated data.

```bash
rosrun holydrone_identification identify.py physics \
  --train /data/flight_train_trimmed.csv \
  --validation /data/flight_validation_trimmed.csv \
  --survey /data/measured_survey.yaml \
  --output /data/gazebo_physics_candidate.yaml
```

Inputs must be contiguous, approximately uniformly sampled **airborne** windows with valid
pose/velocity/IMU and all four RPMs. Trim takeoff/landing/stale intervals; do not concatenate
disjoint windows and pretend they are contiguous. For external CSVs the required columns are:

```text
t,wx,wy,wz,fz,vx,vy,vz,rpm0,rpm1,rpm2,rpm3
```

A known measured mass fixes the absolute force scale. Low-speed, low-rate samples estimate a
common `kT` from `mass * specific_force_z = kT * sum(rotor_speed²)`. Three-axis angular excitation
then fits `I * angular_acceleration + omega × (I*omega) + D*omega = rotor_torque`.
Pure hover does **not** identify inertia. Insufficient rank/conditioning, nonphysical inertia,
stale RPM or absent excitation causes an error rather than fabricated coefficients. If inertia
is measured separately, provide `inertia_diagonal_kg_m2` to reduce the number of unknowns.

Output includes common rotor `kT`, `kQ`, Gazebo `momentConstant = kQ/kT`, aggregate inertia,
angular damping and independent force/angular-acceleration residuals. `passed` requires validation
force RMSE below 10% of vehicle weight and each angular-acceleration RMSE below 2 rad/s².
These are explicit initial acceptance thresholds, not a certification of the aircraft model.

Assumptions: identical vertical rotors, diagonal inertia, rigid body, negligible aerodynamic force
in the selected calibration windows. Mass, center of mass, full off-diagonal inertia, individual
motor differences, translational drag and battery dependence are **not automatically identified**.
The aggregate inertia is not directly the Gazebo `base_link` inertia: subtract/allocate the separately
modeled rotor and sensor contributions before applying it. The tool never edits SDF/PX4 automatically.

### Motor/propeller bench data

```bash
rosrun holydrone_identification identify.py bench \
  --train /data/motor_bench.csv --output /data/motor_candidate.yaml
```

Columns: `t,command_rad_s,omega_rad_s,thrust_n,torque_nm`. Use calibrated speed demands, not raw
PWM labeled as rad/s. Include multiple settled levels and transitions. Outputs provide `kT`, `kQ`,
`momentConstant` and a first-order speed response. Bench output remains `passed: false` until separately
validated; evaluate rising and falling transitions separately before choosing Gazebo's
`timeConstantUp` and `timeConstantDown`. Voltage dependence needs repeated bench runs.

## 2. Gazebo closed-loop acquisition

First calibrate/validate the Gazebo aircraft and match the actual PX4 configuration. Do not run
`policy_fsm`, another setpoint publisher, or another simulator using the same MAVLink ports.
Use a separate ROS master if other projects are running.

```bash
# PX4 airframe/plugin installation follows the parent holydrone README.
export PX4_DIR=/path/to/PX4-Autopilot
roslaunch holydrone_identification sim.launch \
  output:=/absolute/path/id_train gui:=true auto_takeoff:=true
```

Automatic OFFBOARD entry, arming and takeoff are **simulation only**, opt-in via `auto_takeoff`.
The simulated vehicle first stabilizes at 2 m. It then executes short positive/negative pulses
on one channel at a time and returns to position hold between pulses. After the final pulse the vehicle recovers stable hover before declaring completion;
automatic simulation then requests landing and waits for confirmed disarming. Default parameters are in `config/excitation.yaml`; the default `hover_source: feedback` measures median PX4 ATTITUDE_TARGET thrust during stable
position hold and freezes that baseline for each pulse. The launch requests message 83 at 50 Hz;
if it is unavailable, excitation waits and eventually times out. `hover_source: fixed` explicitly
uses `hover_thrust` for a separately calibrated baseline. Do not treat MPC_THR_HOVER as a measured
hover command or enlarge the envelope to hide a wrong baseline. `fault` stays latched; do not resume pulses
until the cause is understood. No RL policy is used.

Run another acquisition with a different `seed` in a separate config file for validation:

```bash
roslaunch holydrone_identification sim.launch \
  output:=/absolute/path/id_validation gui:=false auto_takeoff:=true \
  config:=/absolute/path/validation_excitation.yaml
```

Observe the named state with `rostopic echo /identification/state`; detailed status remains
on `/identification/status`. Transitions and reasons are published on `/identification/events`
and saved by the recorder to `fsm_events.jsonl`. Stop only the excitation via:

```bash
rosservice call /identification/stop '{}'
```

This cancels pulses and holds the current position when feedback is valid; it is not a disarm
command. Invalid feedback while still armed in OFFBOARD requests AUTO.LAND. Mode exit withdraws
setpoints. Ctrl+C ends the nodes; complete landing or transfer control before doing so in real flight.
The automatic sequence checks pose, velocity, rate, tilt, feedback freshness, altitude, displacement,
session time, mode exit and disarming. It checks for competing position/body-rate/local setpoint
publishers and withdraws if another controller appears. The clock watchdog uses wall time, while
pulse timing and command scheduling follow ROS time.

### Active real-flight excitation (explicit opt-in)

Prefer passive data collection and bench experiments first. Configure amplitudes, hover thrust,
altitude/displacement/tilt bounds and session time for the actual aircraft. With pilot-controlled
stable airborne flight and a suitable handover procedure:

```bash
roslaunch holydrone_identification identification_fsm.launch environment:=real \
  enable_real_excitation:=true config:=/data/measured_excitation.yaml
rosservice call /identification/prepare '{}'
# Operator selects OFFBOARD after the position stream is established.
rosservice call /identification/start '{}'
```

The real node never requests arming or OFFBOARD entry. `prepare` requires armed flight in the
altitude envelope; `start` additionally requires stable armed OFFBOARD. Completion holds position
until operator takeover. Do not keep another controller publishing setpoints during this process.
No real-flight suitability is claimed without hardware testing.

## 3. First-order fitting and validation

```bash
rosrun holydrone_identification identify.py response \
  --train /data/id_train/record/samples.csv \
  --validation /data/id_validation/record/samples.csv \
  --output /data/response.yaml
```

Optional QUAD_RL export after validation passes:

```bash
rosrun holydrone_identification identify.py response \
  --train /data/id_train/record/samples.csv \
  --validation /data/id_validation/record/samples.csv \
  --output /data/response.yaml \
  --training-config /path/to/QUAD_RL/configs/ppo_rate_hover.yaml
```

The tool fits `tau * dy/dt + y = gain*u(t-delay) + bias` for roll/pitch/yaw angular velocity
and body-z specific force. It handles zero-order-held commands and individual pulse initial states;
never fits across recovery gaps. CSVs from other sources need `segment,channel,valid` plus
`t,cmd_wx,cmd_wy,cmd_wz,cmd_thrust,wx,wy,wz,fz`. Channels are `roll,pitch,yaw,thrust`.
Only full pulses with valid interior samples and adequate excitation are accepted; invalid edge
samples may be trimmed. Freshness/coverage failures cannot silently become valid training data.

Validation uses a different acquisition (identical files are rejected), checks signal span,
absolute RMSE (0.15 rad/s, 1 m/s²) and normalized RMSE (0.2 of the validation 5–95% span).
Fits at parameter limits fail validation. Delay is effective command-to-measurement delay,
including feedback transport/filtering; it is not uniquely the actuator delay.

`response.yaml` has QUAD_RL-compatible `channels` with `tau_s,delay_s,gain,bias` and a top-level
`passed`. Failed reports remain useful evidence but do not generate a training configuration.
Successful export writes `response.training.yaml`, restricts action bounds to the overlap of training/validation excitation ranges, and disables
`allow_unvalidated_model`. This avoids silently training with 3 rad/s commands after identifying
only 0.2 rad/s pulses. Broader action ranges require broader independent validation. Randomization ranges are deliberately not invented from a single fit.
The one-axis tests validate a local approximation; simultaneous-axis and larger-amplitude
validation is still needed for wider flight envelopes.

## Tests

```bash
source /path/to/catkin_ws/devel/setup.bash
python3 -m unittest discover -s /path/to/holydrone/holydrone_identification/test -v
```

Synthetic tests recover known response/physical parameters and reject reused validation logs,
insufficient excitation and inadequate held-out predictions. Real hardware testing is not implied.

## Explicit identification FSM

`scripts/identification_fsm.py` is the ROS entry; `src/holydrone_identification/fsm.py`
contains the single implementation and allowed transition table. `excite.py` and `excite.launch`
remain compatibility entries. `sim.launch` directly includes `identification_fsm.launch`.

```text
IDLE → HOLD → [TAKEOFF, automatic simulation only] → RECOVER → EXCITE
                                                     ↑         │
                                                     └─────────┘
After the final pulse: RECOVER → COMPLETE
    simulation automatic: COMPLETE → LANDING → FINISHED (actual disarm feedback)
    manual/real: COMPLETE holds position → FINISHED on operator takeover/disarm
Any active failure or stop request: ABORTED (latched until a new prepare request)
```

Services remain `/identification/prepare`, `/identification/start`, `/identification/stop`.
An explicit `/identification/land` service requests landing from an active armed OFFBOARD
experiment, including an aborted experiment with valid feedback. Real flight still never
requests automatic arming or OFFBOARD entry.

`COMPLETE` means all planned pulses finished **and final hover recovery succeeded**.
`FINISHED` means the session has ended; it alone does not mean all pulses were collected.
The `sequence_complete` field distinguishes a completed sequence from an early landing.
`ABORTED` carries a reason and excludes the interrupted pulse from fitting. A new prepare
request assigns fresh segment IDs so multiple attempts cannot merge into a single fitted pulse.
Waiting for FCU hover-thrust feedback continues position streaming and status publication.

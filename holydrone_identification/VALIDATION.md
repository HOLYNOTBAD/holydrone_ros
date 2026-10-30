# Validation — 2026-10-30

Implemented and tested with ROS Noetic, Gazebo Classic 11 and PX4 v1.16.2.
The aircraft assets are holydrone revision `6d6bdd3` (before adding this package).

- Catkin discovers/builds the identification package through the separate source symlink.
- Synthetic response recovery and held-out rejection; synthetic physical inertia/thrust recovery;
  insufficient excitation rejection; bench estimation; aborted-pulse rejection; clock/real-flight
  opt-in/control-ownership/start gates are exercised by the unittest suite.
- Initial Gazebo experiment using a fixed 0.24 thrust baseline aborted on the flight envelope.
  This exposed the difference between the PX4 parameter default and the actual hover command.
- With measured FCU ATTITUDE_TARGET baseline, two independent acquisitions (seeds 100 and 201,
  one pulse pair per channel) completed all four channels and landed/disarmed automatically.
  Observed stable hover thrust was approximately 0.337.
- Training acquisition: QUAD_RL `results/identification_validation/sim_02/record/`.
  Independent validation: `results/identification_validation/sim_03/record/`.

| Channel | tau (s) | delay (s) | gain | bias | Validation RMSE |
|---|---:|---:|---:|---:|---:|
| roll | 0.01010 | 0.02671 | 1.00434 | 0.00119 | 0.06324 rad/s |
| pitch | 0.00740 | 0.03745 | 0.91791 | 0.01182 | 0.06068 rad/s |
| yaw | 0.01962 | 0.07032 | 1.02909 | 0.00649 | 0.08532 rad/s |
| thrust | 0.01955 | 0.02000 | 36.42051 | -2.45974 | 0.13513 m/s² |

All four channels passed the configured local validation thresholds. Predicted hover command:
0.336891. Artifacts: `response_local.yaml` and `response_local.training.yaml` in the same
QUAD_RL results directory. The exported training config is restricted to tested action ranges.

This is **local, small-amplitude SITL identification**, not physical calibration of the real
aircraft and not validation of an RL policy. No physical aircraft or motor bench was operated.
The real-physics pipeline was checked against synthetic data with known parameters. More pulse
pairs, voltage/load conditions and coupled-axis validation are needed before broadening the model.


## Identification FSM refactor

The explicit `IdentificationFSM` replaces the implicit excite-script state management.
Regression tests cover final recovery before completion, landing feedback before FINISHED,
operator takeover, abort latching/control withdrawal, continued hold while awaiting hover
feedback, illegal transitions, and competing setpoint publishers after completion.
The legacy `excite.launch` forwards to the new `identification_fsm.launch`.

Gazebo regression: `results/identification_validation/fsm_01/record/fsm_events.jsonl`
records four EXCITE intervals, final RECOVER → COMPLETE at 30.88 s, LANDING,
and FINISHED at 41.536 s only after PX4 disarming. No ABORTED transition occurred.
The recorder's CSV includes `fsm_state`, `reason`, and `sequence_complete`; all four
completed pulse segments remain readable by the response fitter. 20 unittests pass.

# Third-party provenance

- `gazebo/models/lidar` was copied from PX4-SITL_gazebo-classic commit
  `70683dc759cbd3ebccfa78429b564f8b1fb5d149`.
- The PX4 airframe overlay targets PX4-Autopilot `v1.16.2`, commit
  `54f0455ffcd755534539a7cf33a09a20bf71d29d`. PX4-Autopilot is distributed
  under the BSD 3-Clause license.
- `gazebo/models/gps` and `gazebo/models/fpv_cam` began from PX4 Gazebo Classic
  model conventions and contain project-specific simulation changes.
- `gazebo/worlds/baylands.world` follows the PX4 Gazebo Classic Baylands world,
  with a project-specific flat takeoff pad. The accompanying Baylands model was
  collected from the local Gazebo model cache used by that world; its original
  `model.config` credited Cole Biesemeyer (`cole@osrfoundation.org`).
- The Holydrone CAD-derived STL meshes and the remaining project files follow
  the repository's licensing status. The repository currently does not state a
  repository-wide software or hardware license.

Before redistributing this package independently, review and preserve the
upstream PX4 notices and specify a license for the Holydrone-owned files.

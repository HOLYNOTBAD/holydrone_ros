cmake_minimum_required(VERSION 3.0.2)
project(holydrone)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_package.py)
endif()

catkin_install_python(PROGRAMS
  scripts/camera_frustum_marker.py
  scripts/configure_mavlink_streams.py
  scripts/sync_gazebo_camera_model.py
  scripts/tpv_camera_follow.py
  scripts/uav_mesh_marker.py
  scripts/world_map_alignment_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  aircraft
  gazebo
  launch
  px4
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md THIRD_PARTY.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

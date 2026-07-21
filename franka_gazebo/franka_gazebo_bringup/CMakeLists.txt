cmake_minimum_required(VERSION 3.5)
project(franka_gazebo_bringup)

find_package(ament_cmake REQUIRED)

# Install project launch files and assets
install(
  DIRECTORY
    launch config worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_gazebo_launch.py TIMEOUT 600)
  # test_gazebo_contact_wrench.py is disabled: on Ignition Fortress the
  # gravity-compensated arm drifts into a near-singular pose (DART joint-limit bug,
  # gazebosim/gz-sim#1684) and Fortress folds joint friction into the estimated
  # wrench, so a stable arm and a clean estimate are not both achievable.
endif()

ament_package()

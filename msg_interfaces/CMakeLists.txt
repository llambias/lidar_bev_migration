cmake_minimum_required(VERSION 3.8)
project(lidar_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Add sensor_msgs here

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Buoy.msg"
  "msg/BuoyArray.msg"
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  "msg/AudioData.msg"
  "msg/NavInfo.msg"
  "msg/WamvPose.msg"
  "msg/AprilTagDetection.msg"        # UAV
  "msg/AprilTagDetectionArray.msg"   # UAV
  "msg/CanDetection.msg"             # UAV
  "msg/CanDetectionArray.msg"        # UAV
  "msg/Point.msg"                    # UAV
  "msg/LandingPadDetection.msg"      # UAV
  "msg/RNMarker.msg"                 # UAV
  "msg/RNMarkerDetectionArray.msg"   # UAV
  "msg/Obstacle.msg"                
  "msg/ObstacleList.msg"           
  # "srv/DroneBridge.srv"
  # "srv/TcpSimVideo.srv"
  # "srv/HardwareInterface.srv"        # WAM-V
  # "srv/WamvCrtlBridge.srv"           # jetson-RPi communication bridge
  # "srv/TaskReport.srv"               # Heartbeat
  # "srv/HelipadTargets.srv"
  # "action/FindPortal.action"         # task 0/3
  # "action/PassPortal.action"         # task 0/3
  # "action/FindGate.action"           # task 2
  # "action/PassGate.action"           # task 2
  # "action/FindBuoy.action"           # task 2
  # "action/TurnAroundBuoy.action"     # task 2
  # "action/FindCreature.action"       # task 4
  # "action/PointToCreature.action"    # task 4
  # "action/IdentifyCreature.action"   # task 4
  # "action/TurnAroundCreature.action" # task 4
  # "action/FindLedBuoy.action"        # task 5
  # "action/PointToLedBuoy.action"     # task 5
  # "action/GetColorPattern.action"    # task 5
  # "action/MappingTasks.action"       # task 5
  # "action/GoToWaypoint.action"       # navigation
  # "action/DeployAndSearch.action"    # UAV task 7
  # "action/Task.action"               # For the action manager of the finals
  # "action/ReturnToWamv.action"
  # "action/SearchAndReport.action"    # UAV task 8
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
  DEPENDENCIES sensor_msgs  # Add sensor_msgs as a dependency
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(interfaces_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Vector2.msg"
  "msg/Pose2D.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Mask.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint3DArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/LaneInfo.msg"
  "msg/MotionCommand.msg"
  "msg/TargetPoint.msg"
  "msg/PathPlanningResult.msg"
  "msg/ParkingState.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()


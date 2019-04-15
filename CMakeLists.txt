cmake_minimum_required(VERSION 2.8.3)
project(record_rosbag_oncall)

set(MSG_DEPS
  std_msgs
  sensor_msgs
  geometry_msgs)

find_package(catkin COMPONENTS roscpp message_generation ${MSG_DEPS})

set(MSG_FILES
)

add_message_files(DIRECTORY msgs FILES ${MSG_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp message_runtime ${MSG_DEPS}
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

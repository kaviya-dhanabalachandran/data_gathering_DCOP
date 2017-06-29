set(navigation_IMPLEMENTATIONS
  client/nav_client.cpp
  thread/threadengine.cpp
  lcm/lcmthread.cpp
  lcm/lcmhandler.cpp
  lcm/timestamped_waypoint_list_handler.cpp
  beans/node.cpp
  lcm/protectedmutex.cpp
  agent/HRVOAgent.cpp
  agent/RVO/Agent.cpp
  agent/RVO/Obstacle.cpp
  agent/RVO/KdTree.cpp
  agent/RVO/RVOSimulator.cpp
  agent/Agent.cpp
  agent/HRVO/Agent.cpp
  agent/HRVO/RoadmapVertex.cpp
  agent/HRVO/Vector2.cpp
  agent/HRVO/Obstacle.cpp
  agent/HRVO/HRVOSimulator.cpp
  agent/HRVO/KdTree.cpp
  agent/HRVO/Goal.cpp
  agent/HLAgent.cpp
  agent/ORCAAgent.cpp  )
  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lcm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
if( ARGOS_COMPILE_REAL_SWARMANOID )
  include_directories(/usr/local/angstrom/arm/include/)
  link_directories(
    ${CMAKE_BINARY_DIR}/../../../common/swarmanoid/control_interface
    ${CMAKE_BINARY_DIR}/../../../common/swarmanoid/utility
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/common
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/vision_robot/filtering
    ${CMAKE_SOURCE_DIR}/../../real_robot/swarmanoid/opencv/lib
    ${CMAKE_BINARY_DIR}/../../../real_robot/swarmanoid/footbot
    )
  add_library(navigation STATIC
    ${navigation_IMPLEMENTATIONS}
    )
  target_link_libraries(navigation lcm)

else( ARGOS_COMPILE_REAL_SWARMANOID )
  add_definitions(-fPIC)
  add_definitions(-DFOOTBOT_SIM)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lcm)
  add_library(navigation STATIC
    ${navigation_IMPLEMENTATIONS})
  add_executable(waypointlist_listener lcm/waypointlist_listener.cpp)
  target_link_libraries(waypointlist_listener lcm)
endif( ARGOS_COMPILE_REAL_SWARMANOID )




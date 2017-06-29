set(footbot_baseStation_HEADERS  
  footbot_baseStation.h
  )

set(footbot_baseStation_IMPLEMENTATIONS
  footbot_baseStation.cc
  )

add_definitions(-std=c++11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${CMAKE_SOURCE_DIR}/controllers)

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
  
  add_executable(footbot_baseStation
    ${footbot_baseStation_IMPLEMENTATIONS}
    ${CMAKE_SOURCE_DIR}/main_functions/real_footbot_main.cpp
    )

  target_link_libraries(footbot_baseStation 
    argos2_real_robot_swarmanoid_footbot
    argos2_real_robot_swarmanoid_common
    argos2_common_control_interface 
    argos2_common_utility
    lcm
    #navigation
    )

else( ARGOS_COMPILE_REAL_SWARMANOID )
  
  add_definitions(-DFOOTBOT_SIM)
  
  #include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/client)
  #include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/lcm)
  #link_directories(${CMAKE_BINARY_DIR}/lib/utils)
  
  add_library(footbot_baseStation MODULE ${footbot_baseStation_IMPLEMENTATIONS})
  
  target_link_libraries(footbot_baseStation 
                        lcm 
                        #navigation
                        )

endif( )

#test 2
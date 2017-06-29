set(footbot_relay_HEADERS  
  utils.h
  footbot_relay.h
  probabilityCalculation.h
  grid.h
  )

set(footbot_relay_IMPLEMENTATIONS
  utils.cpp
  footbot_relay.cc
  probabilityCalculation.cpp
  grid.cpp
 )

##find_package(Boost 1.54 REQUIRED COMPONENTS filesystem iostreams)
##find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH /usr/local/include/eigen3 ${CMAKE_SOURCE_DIR}/cmake)
##find_package(Eigen3 REQUIRED)

add_definitions(-std=c++11)

include_directories(/usr/local/include/eigen3)
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
  add_executable(footbot_relay
    ${footbot_relay_IMPLEMENTATIONS}
    ${CMAKE_SOURCE_DIR}/main_functions/real_footbot_main.cpp
    )

  #set_property(TARGET footbot_relay PROPERTY CXX_STANDARD 11)
  #set_property(TARGET footbot_relay PROPERTY CXX_STANDARD_REQUIRED ON)
  

  target_link_libraries(footbot_relay
    argos2_real_robot_swarmanoid_footbot
    argos2_real_robot_swarmanoid_common
    argos2_common_control_interface 
    argos2_common_utility
    lcm
    navigation
    ${OpenCV_LIBS}
    )

else( ARGOS_COMPILE_REAL_SWARMANOID )

  add_definitions(-DFOOTBOT_SIM)

  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/client)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/lcm)
 
  add_library(footbot_relay 
    MODULE 
    ${footbot_relay_IMPLEMENTATIONS})

  target_link_libraries(footbot_relay lcm navigation ${OpenCV_LIBS})

endif( )

#test 2

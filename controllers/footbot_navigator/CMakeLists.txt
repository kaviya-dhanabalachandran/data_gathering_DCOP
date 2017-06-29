
set(footbot_navigator_HEADERS  
  footbot_navigator.h
  )

set(footbot_navigator_IMPLEMENTATIONS
  footbot_navigator.cc
  )

#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/navigation/lcm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${CMAKE_SOURCE_DIR}/controllers)
#include_directories( ${CMAKE_SOURCE_DIR}/controllers/rvo_nav/client)
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
  add_executable(footbot_navigator
    ${footbot_navigator_IMPLEMENTATIONS}
    ${CMAKE_SOURCE_DIR}/main_functions/real_footbot_main.cpp
    )

  #ADD_LIBRARY(lcm_static STATIC IMPORTED)
  #SET_TARGET_PROPERTIES(lcm_static PROPERTIES 
  #IMPORTED_LOCATION /usr/local/angstrom/arm/arm-angstrom-linux-gnuea/lib/liblcm.la)

  target_link_libraries(footbot_navigator 
    argos2_real_robot_swarmanoid_footbot
    argos2_real_robot_swarmanoid_common
    argos2_common_control_interface 
    argos2_common_utility
    lcm
    navigation
    )

else( ARGOS_COMPILE_REAL_SWARMANOID )
  add_definitions(-DFOOTBOT_LQL_SIM)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/client)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav)
  include_directories(${CMAKE_SOURCE_DIR}/controllers/rvo_nav/lcm)
  #link_directories(${CMAKE_BINARY_DIR}/lib/utils)
  add_library(footbot_navigator 
    MODULE 
    ${footbot_navigator_IMPLEMENTATIONS})
  target_link_libraries(footbot_navigator lcm navigation)

endif( )

#test 2




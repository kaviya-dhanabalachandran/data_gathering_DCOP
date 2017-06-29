include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-fPIC)
add_library(footbot_central_controller MODULE
  footbot_central.cpp
)

link_directories(/usr/local/lib)
target_link_libraries(footbot_central_controller
  -pthread -L/usr/local/lib -llcm -lgthread-2.0 -lglib-2.0
  navigation
  /usr/local/lib/liblcm.so
  #argos2_simulator_dynamic_linking
  )

cmake_minimum_required(VERSION 2.8)

project(scenenet_room_generator)

set(Chrono_DIR "/path/to/your/chrono/build/cmake")
find_package(GSL REQUIRED)
find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------
include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIRS})

add_executable(scenenet_room_generator scenenet_room_generator.cpp)

set_target_properties(scenenet_room_generator PROPERTIES 
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

target_link_libraries(scenenet_room_generator ${CHRONO_LIBRARIES} ${GSL_LIBRARIES})

# -DCMAKE_BUILD_TYPE=Debug
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

project( Renderer )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fext-numeric-literals -D_FORCE_INLINES -fPIC")

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

# Platform configuration vars
INCLUDE(SetPlatformVars)

FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS   "-arch=compute_61 -code=sm_61"  "--use_fast_math" "-O3" "-lineinfo" "-Xcompiler")
SET(CUDA_HOST_COMPILER gcc-5)
FIND_PACKAGE(CVD REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OptiX REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(ASSIMP REQUIRED)

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

INCLUDE_DIRECTORIES(
  ${CUDA_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OptiX_INCLUDE}
  ${ASSIMP_INCLUDE}
  ${CVD_INCLUDE_DIR}
)


# SUtil compilation
set(PTX_DIR "${CMAKE_BINARY_DIR}/ptx" CACHE PATH "Path to where the samples look for the PTX code.")
set(RESOURCES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/Resources" CACHE PATH "Path to the samples directory.")

set(CUDA_GENERATED_OUTPUT_DIR ${PTX_DIR})

if ( USING_GNU_C AND NOT APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DM_PI=3.14159265358979323846" )
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/locationConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/locationConfig.h @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/Util/sutil/sampleConfig.h.in ${CMAKE_CURRENT_BINARY_DIR}/sampleConfig.h @ONLY)

# Path to sutil.h that all the samples need
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/Util/sutil
                    ${OptiX_INCLUDE}/optixu
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CUDA_INCLUDE_DIRS} 
)

# Our sutil library.  The rules to build it are found in the subdirectory.
add_subdirectory(src/Util/sutil)

# Build our shaders
file(GLOB renderer_ptx_srcs src/Renderer/PhotonMapping/*.cu)
file(GLOB geometry_ptx_srcs src/Geometry/*.cu)
file(GLOB material_ptx_srcs src/Material/*.cu)
file(GLOB generator_ptx_srcs src/RayGenerators/*.cu)
file(GLOB camera_ptx_srcs src/CameraModels/*.cu)
file(GLOB util_ptx_srcs src/Util/Mesh/*.cu)

file(GLOB renderer_srcs src/Renderer/*.cpp)
file(GLOB renderer_cuda_srcs src/Renderer/*.cu)
file(GLOB scene_srcs src/Scene/*.cpp)
file(GLOB geometry_srcs src/Geometry/*.cpp)
file(GLOB material_srcs src/Material/*.cpp)
file(GLOB generator_srcs src/RayGenerators/*.cpp)
file(GLOB camera_srcs src/CameraModels/*.cpp)
file(GLOB math_srcs src/Math/*.cpp)
file(GLOB trajectory_srcs src/trajectoryGenerator/*.cpp)


CUDA_COMPILE_PTX(ptx_shaders ${renderer_ptx_srcs}
                             ${geometry_ptx_srcs}
                             ${material_ptx_srcs}
                             ${generator_ptx_srcs}
                             ${camera_ptx_srcs}
                             ${util_ptx_srcs}
)

CUDA_COMPILE(renderer_cuda_objs ${renderer_cuda_srcs})
link_libraries(${CUDA_LIBRARIES}
               ${optix_LIBRARY}
               ${ASSIMP_LIBRARY}
               ${CVD_LIBRARY}
               sutil_sdk
              )

add_custom_target(shaders DEPENDS ${ptx_shaders})

# We need C++14 for the latest CVD
SET(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -fext-numeric-literals -D_FORCE_INLINES -fPIC")
add_executable(scenenet_render src/scenenet_render.cpp 
                                         ${renderer_cuda_objs}
                                         ${renderer_srcs}
                                         ${scene_srcs}
                                         ${geometry_srcs}
                                         ${material_srcs}
                                         ${generator_srcs}
                                         ${camera_srcs}
                                         ${math_srcs}
                                         ${trajectory_srcs})
add_dependencies(scenenet_render shaders)

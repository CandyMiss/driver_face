cmake_minimum_required(VERSION 3.0.2)
project(driver_face)
set(PROJ_NAME driver_face)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

find_package(CUDA REQUIRED)

set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)

include_directories(/usr/include)
include_directories(/usr/include/glib-2.0)
include_directories(/usr/lib/glib-2.0/include)
# 区分嵌入式平台环境
include_directories(${PROJECT_SOURCE_DIR}/include)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("embed_platform on")
    include_directories(/usr/local/cuda/targets/aarch64-linux/include)
    link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
    link_directories(/usr/lib/aarch64-linux-gnu)    # 字体渲染库，libfreetype的位置
    include_directories(/usr/include/gstreamer-1.0)
    include_directories(/usr/lib/aarch64-linux-gnu/glib-2.0/include)
else()
    message("embed_platform off")
    include_directories(/usr/local/cuda/include)
    link_directories(/usr/local/cuda/lib64)
    link_directories(/usr/lib/x86_64-linux-gnu/)
    include_directories(/usr/lib/x86_64-linux-gnu/gstreamer-1.0)
    include_directories(/usr/lib/x86_64-linux-gnu/gstreamer-1.0/include)
    include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0)
    include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include)
endif()

# 为了防止对 #pragma 语句的报警，加上 -Wno-unknown-pragmas 设置
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unknown-pragmas -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

# 不明白工程里的问题，这里凡是使用了 CUDA代码，都要编译为动态链接库才能使用
cuda_add_library(yololayer SHARED src/yolov5/yololayer.cu)
cuda_add_library(face_similarity SHARED ${PROJECT_SOURCE_DIR} src/arcface/face_similarity.cu)

add_message_files(
  FILES
  ResultMsg.msg
  FaceRecMsg.msg
 )
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES driver_face
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_generation message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/driver_face.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/driver_face_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_executable(Decoder src/decode.cpp)
target_link_libraries(Decoder ${catkin_LIBRARIES})
add_dependencies(Decoder ${PROJECT_NAME}_generate_messages_cpp)

add_executable(Analyzer src/analyze.cpp
        src/utils/logging.h
        src/utils/utils.h
        src/utils/dataset.h src/utils/dataset.cpp
        src/utils/cfg_params.h src/utils/cfg_params.cpp
        src/pfld/pfld.h src/pfld/pfld.cpp
        src/pfld/face_gesture.h src/pfld/face_gesture.cpp
        src/data/results.h src/data/results.cpp
        src/yolov5/common.h src/yolov5/common.cpp
        src/yolov5/yololayer.h src/yolov5/yololayer.cu
        src/yolov5/yolov5.h src/yolov5/yolov5.cpp)

target_link_libraries(Analyzer ${catkin_LIBRARIES})
add_dependencies(Analyzer ${PROJECT_NAME}_generate_messages_cpp)

add_executable(Renderer src/render.cpp
        src/font/CvxText.cpp src/font/CvxText.h)
target_link_libraries(Renderer ${catkin_LIBRARIES})
add_dependencies(Renderer ${PROJECT_NAME}_generate_messages_cpp)

add_executable(Identifier src/identify.cpp
        src/utils/cfg_params.h src/utils/cfg_params.cpp
        src/data/results.h src/data/results.cpp
        src/pfld/pfld.h src/pfld/pfld.cpp
        src/pfld/face_gesture.h src/pfld/face_gesture.cpp
        src/arcface/prelu.h src/arcface/prelu.cu
        src/arcface/arcface.cpp src/arcface/arcface.h
        src/arcface/draw_mask.cpp src/arcface/draw_mask.h
        src/arcface/face_similarity.cu
        src/database/SqliteOp.cpp src/database/SqliteOp.h
        src/yolov5/common.h src/yolov5/common.cpp)
target_link_libraries(Identifier ${catkin_LIBRARIES})
add_dependencies(Identifier ${PROJECT_NAME}_generate_messages_cpp)

find_package(OpenCV)
include_directories(OpenCV_INCLUDE_DIRS)
target_link_libraries(Decoder ${OpenCV_LIBS})
target_link_libraries(Analyzer ${OpenCV_LIBS})
target_link_libraries(Renderer ${OpenCV_LIBS})
target_link_libraries(Identifier ${OpenCV_LIBS})

target_link_libraries(Analyzer nvinfer)
target_link_libraries(Analyzer cudart)
target_link_libraries(Analyzer yololayer)

include_directories(/usr/lib)
include_directories(/usr/lib/glib-2.0/include)
include_directories(/usr/lib/x86_64-linux-gnu -lgio-2.0)
target_link_libraries(Analyzer glib-2.0)
target_link_libraries(Analyzer gobject-2.0)
target_link_libraries(Analyzer gmodule-2.0)
target_link_libraries(Analyzer gstreamer-1.0)
target_link_libraries(Analyzer gstbase-1.0)
target_link_libraries(Analyzer gstapp-1.0)
target_link_libraries(Analyzer gstnet-1.0)

target_link_libraries(Identifier nvinfer)
target_link_libraries(Identifier cudart)
target_link_libraries(Identifier yololayer)
target_link_libraries(Identifier face_similarity)
target_link_libraries(Identifier sqlite3)

include_directories(/usr/include/freetype2)
include_directories(/usr/include/libxml2)
target_link_libraries(Renderer freetype)
target_link_libraries(Analyzer xml2)
target_link_libraries(Identifier xml2)


# find_package (Threads)
# target_link_libraries (Analyzer ${CMAKE_THREAD_LIBS_INIT})

add_definitions(-O2 -pthread)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_driver_face.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

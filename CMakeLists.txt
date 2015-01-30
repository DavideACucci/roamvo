cmake_minimum_required(VERSION 2.8.3)
project(roamvo)

find_package(catkin REQUIRED COMPONENTS roscpp 
                                        tf 
                                        tf_conversions                                         
                                        roamfree
                                        libviso2_matcher_msgs
                                        )

#this is needed if no system level FindEigen3.cmake is available
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
                                        
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

catkin_package()

set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

include_directories(include 
                ${catkin_INCLUDE_DIRS}
		            ${EIGEN3_INCLUDE_DIR}
		            ${Boost_INCLUDE_DIRS}  
)

add_executable(roamvo_node
	           src/VisualOdometryNode.cpp
	           src/ImuHandler.cpp)

target_link_libraries(roamvo_node
                      ${catkin_LIBRARIES})

target_link_libraries(roamvo_node ${catkin_LIBRARIES})

add_dependencies(roamvo_node ${catkin_EXPORTED_TARGETS})
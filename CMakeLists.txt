cmake_minimum_required(VERSION 3.5)
project(cloud2range)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        cv_bridge
                        pcl_ros
                        pcl_conversions
                        sensor_msgs)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(OpenMP)

catkin_package(# INCLUDE_DIRS include
               # LIBRARIES
               CATKIN_DEPENDS
               roscpp
               cv_bridge
               pcl_ros
               pcl_conversions
               sensor_msgs
               DEPENDS)

add_library(${PROJECT_NAME} src/cloud2range_node.cpp src/range2cloud_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Boost::timer)
# if(OpenMP_CXX_FOUND) target_link_libraries(${PROJECT_NAME} PUBLIC
# OpenMP::OpenMP_CXX) endif()

add_executable(cloud2range_node src/cloud2range_main.cpp)
target_link_libraries(cloud2range_node PRIVATE ${PROJECT_NAME})

add_executable(range2cloud_node src/range2cloud_main.cpp)
target_link_libraries(range2cloud_node PRIVATE ${PROJECT_NAME})

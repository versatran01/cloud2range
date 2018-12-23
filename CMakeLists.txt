cmake_minimum_required(VERSION 3.5)
project(cloud2range)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        cv_bridge
                        image_transport
                        pcl_ros
                        pcl_conversions
                        sensor_msgs)

catkin_package(# INCLUDE_DIRS include
               # LIBRARIES
               CATKIN_DEPENDS
               roscpp
               cv_bridge
               image_transport
               pcl_ros
               pcl_conversions
               sensor_msgs
               DEPENDS)

add_executable(cloud2range_node src/cloud2range_node.cpp)
target_include_directories(cloud2range_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(cloud2range_node PRIVATE ${catkin_LIBRARIES})

add_executable(range2cloud_node src/range2cloud_node.cpp)
target_include_directories(range2cloud_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(range2cloud_node PRIVATE ${catkin_LIBRARIES})

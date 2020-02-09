cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()



### Original CMakeLists.txt file provided by https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html


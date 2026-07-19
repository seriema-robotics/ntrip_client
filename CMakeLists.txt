cmake_minimum_required(VERSION 3.5)
project(ntrip_client)

# Check if catkin is available
find_package(catkin QUIET)

if(catkin_FOUND)
  message(STATUS "Building ntrip_client as a ROS 1 Catkin package")
  find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    rtcm_msgs
    nmea_msgs
  )
  
  catkin_package(
    CATKIN_DEPENDS
      rospy
      std_msgs
      rtcm_msgs
      nmea_msgs
  )
  
  # Install scripts
  catkin_install_python(PROGRAMS
    scripts/ntrip_ros.py
    scripts/ntrip_serial_device_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  # Set up python modules
  catkin_python_setup()
  
  # Install launch files
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

else()
  # Look for ament_cmake
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  message(STATUS "Building ntrip_client as a ROS 2 Ament package")
  
  # Install the Python package (from src/ntrip_client)
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "src/${PROJECT_NAME}")
  
  # Install executable scripts
  install(PROGRAMS
    scripts/ntrip_ros2.py
    scripts/ntrip_ros2_base.py
    scripts/ntrip_serial_device_ros2.py
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # Install launch files
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_package()
endif()
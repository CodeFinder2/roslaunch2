cmake_minimum_required(VERSION 2.8.3)
project(roslaunch2)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)
catkin_package()
catkin_python_setup()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/roslaunch2
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/roslaunch2_auto_completion.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Note: No need to install the files in src/${PROJECT_NAME} manually as this is done automatically.

execute_process(COMMAND "/home/cc/ee106a/fa22/class/ee106a-aaz/ros_workspaces/106a-planty-bot/build/turtlebot3_automatic_parking_vision/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106a/fa22/class/ee106a-aaz/ros_workspaces/106a-planty-bot/build/turtlebot3_automatic_parking_vision/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

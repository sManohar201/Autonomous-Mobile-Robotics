# CMake generated Testfile for 
# Source directory: /home/sam/work/RoboticsND/jackal/jackal_description
# Build directory: /home/sam/work/RoboticsND/jackal/jackal_description/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_jackal_description_roslaunch-check_launch_description.launch "/home/sam/work/RoboticsND/jackal/jackal_description/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/sam/work/RoboticsND/jackal/jackal_description/cmake-build-debug/test_results/jackal_description/roslaunch-check_launch_description.launch.xml" "--return-code" "/opt/clion/bin/cmake/linux/bin/cmake -E make_directory /home/sam/work/RoboticsND/jackal/jackal_description/cmake-build-debug/test_results/jackal_description" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sam/work/RoboticsND/jackal/jackal_description/cmake-build-debug/test_results/jackal_description/roslaunch-check_launch_description.launch.xml\" \"/home/sam/work/RoboticsND/jackal/jackal_description/launch/description.launch\" ")
set_tests_properties(_ctest_jackal_description_roslaunch-check_launch_description.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sam/work/RoboticsND/jackal/jackal_description/CMakeLists.txt;8;roslaunch_add_file_check;/home/sam/work/RoboticsND/jackal/jackal_description/CMakeLists.txt;0;")
subdirs("gtest")

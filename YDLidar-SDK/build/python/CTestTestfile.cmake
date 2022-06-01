# CMake generated Testfile for 
# Source directory: /home/zcy/YDLidar-SDK-master/python
# Build directory: /home/zcy/YDLidar-SDK-master/build/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/usr/bin/python" "/home/zcy/YDLidar-SDK-master/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/home/zcy/smartcar_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:/home/zcy/YDLidar-SDK-master/build/python")
subdirs("examples")

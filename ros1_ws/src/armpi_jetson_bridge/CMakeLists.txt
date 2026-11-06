cmake_minimum_required(VERSION 3.0.2)
project(armpi_jetson_bridge)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/armpi_to_jetson.py scripts/encoder_diagnostic.py scripts/wheel_odometry.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES scripts/encoder_reader.py scripts/wheel_math.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

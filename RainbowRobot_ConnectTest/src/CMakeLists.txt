cmake_minimum_required(VERSION 3.10)
project(rb_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(rb_test)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  rb_test/rb_modbus_bridge.py
  rb_test/motion_gui_runner.py
  rb_test/motion_executor.py
  rb_test/test_cobot_publisher.py
  rb_test/sensor_gui.py
  rb_test/rb_io_bridge.py
  rb_test/rb_io_bridge_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.0.2)
project(action_server_monitor)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/action_server_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  catkin_install_python(PROGRAMS test/mock_client.py test/mock_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
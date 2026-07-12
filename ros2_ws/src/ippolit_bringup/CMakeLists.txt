cmake_minimum_required(VERSION 3.8)
project(ippolit_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  systemd
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/deploy.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

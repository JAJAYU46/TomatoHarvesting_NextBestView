# Install script for directory: /home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/src/realsense-ros/realsense2_camera

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/install/realsense2_camera")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera" TYPE EXECUTABLE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/realsense2_camera_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/realsense2_camera/realsense2_camera_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/CMakeFiles/realsense2_camera_node.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/librealsense2_camera.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so"
         OLD_RPATH "/opt/ros/humble/lib/x86_64-linux-gnu:/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/install/realsense2_camera_msgs/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librealsense2_camera.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/src/realsense-ros/realsense2_camera/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE DIRECTORY FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/src/realsense-ros/realsense2_camera/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE DIRECTORY FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/src/realsense-ros/realsense2_camera/examples")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/realsense2_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/realsense2_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/environment" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_index/share/ament_index/resource_index/packages/realsense2_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rclcpp_components" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_index/share/ament_index/resource_index/rclcpp_components/realsense2_camera")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/cmake" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/cmake" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/cmake" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera/cmake" TYPE FILE FILES
    "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_core/realsense2_cameraConfig.cmake"
    "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/ament_cmake_core/realsense2_cameraConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/realsense2_camera" TYPE FILE FILES "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/src/realsense-ros/realsense2_camera/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
  file(WRITE "/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/build/realsense2_camera/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

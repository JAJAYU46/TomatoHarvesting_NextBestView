cmake_minimum_required(VERSION 3.8)
project(my_robot_nbv_cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#for octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES}) #不知為啥有這行就會出錯
#for octomap msg
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#for Sdf_Model
#==================
set(Open3D_DIR ${HOME}/open3d_install/lib/cmake/Open3D)
set(Open3D_INCLUDE_DIRS ${HOME}/open3d_install/include)
# set(Open3D_CORE_DIRS ${HOME}/open3d_install/core)


# list(APPEND CMAKE MODULEPATH ${PROJECT_SOURCE_DIR}/cmake)

#  Open3D
find_package(Open3D REQUIRED)
include_directories(${Open3D_INCLUDE_DIRS})
# include_directories(${Open3D_CORE_DIRS})
# Manually specify the include directory
# include_directories(/home/jajayu/open3d_install/include)
# Tell CMake where to find the Open3D headers
# include_directories(${HOME}/open3d_install/include)

# find_package(Open3D HINTS REQUIRED)
# list(APPEND Open3D_LIBRARIES dl)
list(APPEND Open3D_LIBRARIES Open3D::Open3D)
link_directories(${Open3D_LIBRARY_DIRS})
# if (Open3D_FOUND)
#     message(STATUS "Found 0pen3D ${Open3D_VERSI0N}")
#     link_directories(${Open3D_LIBRARY_DIRS})

# endif()
if (Open3D_FOUND)
    message(STATUS "Found Open3D libraries: ${Open3D_LIBRARIES}")
else()
    message(STATUS "Open3D not found")
endif()



#==================
# find_package(Open3D REQUIRED)
# include_directories(${Open3D_INCLUDE_DIRS})
# link_directories(${Open3D_LIBRARY_DIRS})


#create an executable
add_executable(nbvc_ray_generation src/nbvc_ray_generation.cpp) #要執行的node名稱/ 檔案位置(相對於現在這個package)
add_executable(SdfModel src/SdfModel.cpp) #要執行的node名稱/ 檔案位置(相對於現在這個package)


#####
ament_target_dependencies(nbvc_ray_generation #msg的是加這種的
  rclcpp 
  octomap_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
)# ${OCTOMAP_LIBRARIES}) #說這個executable 裡面有用到(dependency)rclcpp這個package
TARGET_LINK_LIBRARIES(nbvc_ray_generation #好像自己額外的要用這個加才可以
    ${OCTOMAP_LIBRARIES}
)

# install (TARGETS #告訴這個node 是在這個package 定義的
#   nbvc_ray_generation
# 	DESTINATION lib/${PROJECT_NAME} #要到那個workspace的include file 裡面找到這個package/lib file 會看到那個executable node
# )

###  SdfModel##
ament_target_dependencies(SdfModel #msg的是加這種的
  rclcpp 
  std_msgs
)# ${OCTOMAP_LIBRARIES}) #說這個executable 裡面有用到(dependency)rclcpp這個package
TARGET_LINK_LIBRARIES(SdfModel #好像自己額外的要用這個加才可以
    ${Open3D_LIBRARIES}
)

# install (TARGETS #告訴這個node 是在這個package 定義的
#   SdfModel
# 	DESTINATION lib/${PROJECT_NAME} #要到那個workspace的include file 裡面找到這個package/lib file 會看到那個executable node
# )

# Install the targets
install(TARGETS
  nbvc_ray_generation
  SdfModel
  DESTINATION lib/${PROJECT_NAME}
)








if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

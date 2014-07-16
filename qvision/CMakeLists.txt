cmake_minimum_required(VERSION 2.8.3)
project(qvision)

find_package(catkin REQUIRED sensor_msgs image_geometry cv_bridge roscpp std_msgs image_transport pcl_conversions pcl_ros)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)


include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_UI(QT_FORMS_HPP 
  res/SimpleWindow.ui
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/qvision
)

find_package(OpenCV REQUIRED)

QT4_WRAP_CPP(MOC_FILES
   include/qvision/SimpleWindow.h
   include/qvision/FrameLabel.h
   include/qvision/ImageConverter.h
   include/qvision/rosThread.h
   )

add_library( ${PROJECT_NAME}
   src/SimpleWindow.cpp
   src/FrameLabel.cpp
   src/ImageConverter.cpp
   src/rosThread.cpp
   ${QT_FORMS_HPP}
   ${MOC_FILES}
   )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
     ${PCL_COMMON_INCLUDE_DIRS}
     ${PCL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy python_qt_binding #  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM
	${PCL_COMMON_INCLUDE_DIRS}
)
find_package(OpenCV REQUIRED)
find_package(cvBlob REQUIRED)


add_executable(qvision_main src/main.cpp)
target_link_libraries(qvision_main ${PROJECT_NAME} ${QT_LIBRARIES} ${OpenCV_LIBS} ${cvBlob_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})






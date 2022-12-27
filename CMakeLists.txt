cmake_minimum_required(VERSION 3.5)
project(calibration_learning)

find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# camera calibration
# add_executable(monocular1 camera/monocular1.cpp)
# target_link_libraries(monocular1 ${OpenCV_LIBS})
# add_executable(monocular2 camera/monocular2.cpp)
# target_link_libraries(monocular2 ${OpenCV_LIBS})
# add_executable(stereo camera/stereo.cpp)
# target_link_libraries(stereo ${OpenCV_LIBS} ${PCL_LIBRARIES})

# cam_lidar calibration
add_executable(calibration cam_lidar/calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${PCL_LIBRARIES})
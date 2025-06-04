# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_Planning_DroneWaypoints)

add_library(${FILTER_NAME}
	src/CDroneWaypointsFilter.h
	src/CDroneWaypointsFilter.cpp
    src/CDroneMotionPrimitives.h
	src/CDroneMotionPrimitives.cpp)

target_link_libraries(${FILTER_NAME}
	PUBLIC
		CycCore
		CycToolkit
        libconfig++
        libcsv
        ${CyC_OPENCV_WORLD})

add_executable(tu_WaypointsPlanner
		src/tu_WaypointsPlanner.cpp)

target_include_directories(tu_WaypointsPlanner
	PUBLIC
		${CMAKE_SOURCE_DIR}/3rdParty/qt-5.12.2/${CyC_LIBS_BINARY_KEY}/include)

target_link_libraries(tu_WaypointsPlanner
    PUBLIC
        ${FILTER_NAME}
        CycCore
        CycToolkit
        ${CyC_OPENCV_WORLD}
        octomap
		octomath
		glut64)

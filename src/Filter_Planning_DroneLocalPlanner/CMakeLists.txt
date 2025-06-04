# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_Planning_DroneLocalPlanner)

add_library(${FILTER_NAME}
	src/CDroneLocalPlannerFilter.h
	src/CDroneLocalPlannerFilter.cpp
	src/setg/CSETG.h
	src/setg/CSETG.cpp)

target_link_libraries(${FILTER_NAME}
	PUBLIC
		CycCore
		CycToolkit
        libconfig++
        libcsv
        ${CyC_OPENCV_WORLD})

add_executable(tu_DroneLocalPlanner
	src/tu_DroneLocalPlanner.cpp)

target_link_libraries(tu_DroneLocalPlanner
	${FILTER_NAME}
	${CyC_OPENCV_WORLD}
    libconfig++
    libcsv)
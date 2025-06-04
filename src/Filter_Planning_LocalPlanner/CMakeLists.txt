# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_Planning_LocalPlanner)

add_library(${FILTER_NAME}
	src/CLocalPlannerFilter.h
	src/CLocalPlannerFilter.cpp
	src/planners/CPlannerCMNST.h
	src/planners/CPlannerCMNST.cpp)

target_link_libraries(${FILTER_NAME}
	PUBLIC
		CycToolkit
        libconfig++
        libcsv
        ${CyC_OPENCV_WORLD})

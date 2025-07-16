// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <iostream>
#include "CyC_TYPES.h"
#include "plan/CWaypointsPlanner.h"
#include "os/CFileUtils.h"
#include <opencv2/opencv.hpp>
/*
#include <QApplication>
#include <QThread>
#include <octomap\octomap.h>
#include "ViewerGui.h"
*/
std::unique_ptr<CWaypointsPlanner> waypoints_planner;

void showUsage()
{
    printf("\nUsage:\n"
        "tu_WaypointsPlanner [options] map_file.csv\n\n"
        "Options:\n"
        "eg: tu_WaypointsPlanner ../etc/missions/drone/markers_map.csv\n");
    exit(1);
}

int main(int argc, char** argv)
{
    // Do not use scientific notation
    std::cout << std::fixed;

    if (argc < 2)
    {
        showUsage();
        return EXIT_SUCCESS;
    }

    waypoints_planner = std::make_unique<CWaypointsPlanner>(argv[1]);

    CycLandmarks map = waypoints_planner->getLandmarks();
    std::cout << "Number of paths: " << map.size() << std::endl;
    for (const auto& path : map)
    {
        std::cout << "\t[" << path.first << "]: " << path.second.waypoints.size() << " waypoints" << std::endl;
        //std::cout << "\t\t" << path.second.pose.translation_3x1().x() << "\t" << path.second.pose.translation_3x1().y() << std::endl;
        for (const auto& w: path.second.waypoints)
            std::cout << "\t\t" << w.x() << "\t" << w.y() << std::endl;
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}

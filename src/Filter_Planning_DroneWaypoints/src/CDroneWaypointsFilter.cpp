// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CDroneWaypointsFilter.h"

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CDroneWaypointsFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CDroneWaypointsFilter(_params);
}

CDroneWaypointsFilter::CDroneWaypointsFilter(CycDatablockKey key) :
    CCycFilterBase(key)
{
    // Assign the output data type
    setFilterType("CyC_DRONE_WAYPOINTS_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_LANDMARKS;
}

CDroneWaypointsFilter::CDroneWaypointsFilter(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
    // Assign the output data type
    setFilterType("CyC_DRONE_WAYPOINTS_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_LANDMARKS;
}

CDroneWaypointsFilter::~CDroneWaypointsFilter()
{
    if (m_bIsEnabled)
        disable();
}

bool CDroneWaypointsFilter::enable()
{
    for (const auto& source : getInputSources())
    {
        if (source.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc(""))
        {
            m_pInputArucoDetector = source.pCycFilter;
        }
        else if (source.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc(""))
        {
            m_pInputDroneTerminal = source.pCycFilter;
        }
        else
        {
            log_error("CDroneWaypointsFilter: Input filter {}-{} cannot be used as input.",
                source.pCycFilter->getFilterKey().nCoreID,
                source.pCycFilter->getFilterKey().nFilterID);
        }
    }

    // Check if the waypoints_map file exists
    if (m_pInputDroneTerminal != nullptr)
    {
        log_info("Using drone terminal for waypoints_map");
    }
    else
    {
        std::string waypoints_map;
        if (!m_CustomParameters["waypoints_map"].empty())
        {
            waypoints_map = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("waypoints_map"));

            if (!CFileUtils::FileExist(waypoints_map.c_str()))
            {
                log_error("ERROR waypoints_map file does not exist. Disabling the CDroneWaypointsFilter filter.");
                return false;
            }
        }
        else
        {
            log_error("ERROR waypoints_map file undefined. Disabling the CDroneWaypointsFilter filter.");
            return false;
        }

        // Load waypoints map
        m_WaypointsPlanner.loadWaypoints(waypoints_map);
    }

    log_info("CDroneWaypointsFilter::enable() successful");

    m_bIsEnabled = true;
    return m_bIsEnabled;
}

bool CDroneWaypointsFilter::disable()
{
    if (isRunning())
        stop();

    m_bIsEnabled = false;
    return true;
}

bool CDroneWaypointsFilter::process()
{
    bool bReturn = false;
    CycReferenceSetPoints ref_setpoints;

    if (m_pInputDroneTerminal != nullptr)
    {
        const auto tsDroneTerminal = m_pInputDroneTerminal->getTimestampStop();
        if (tsDroneTerminal > m_prevTsDroneTerminal)
        {
            m_prevTsDroneTerminal = tsDroneTerminal;
            CycTerminalCommand term;
            if (m_pInputDroneTerminal->getData(term) && !term.landmarks.empty())
            {
               m_WaypointsPlanner.loadWaypoints(term.landmarks);
            }
        }
    }
    
    ////const auto tsArucoDetection = m_pInputArucoDetector->getTimestampStop();
    ////if (tsArucoDetection > m_prevTsArucoDetection)
    //{
    //    //m_prevTsArucoDetection = tsArucoDetection;

    //    CycPoses aruco_poses;
    //    aruco_poses.emplace_back(CPose(0));
    //    //if (m_pInputArucoDetector->getData(aruco_poses))
    //    {
    //        //spdlog::info("Aruco markers no: {}", aruco_poses.size());

    //        // Get waypoints for the first detected marker, if any
    //        if (aruco_poses.size() > 0)
    //        {
    //            //spdlog::info("Marker id: {}", aruco_poses[0].getID());

    //            //std::vector<Eigen::Vector4f> waypts;
    //            CycLandmark landmark;
    //            if (m_WaypointsPlanner.getLandmark(aruco_poses[0].getID(), landmark))
    //            {
    //                for (const auto& pt : landmark.waypoints)
    //                    ref_setpoints.ref.emplace_back(pt);

    //                ref_setpoints.id = aruco_poses[0].getID();

    //                bReturn = true;
    //            }
    //        }
    //    }
    //}

    for (const auto& landmark : m_WaypointsPlanner.getLandmarks())
    {
        for (const auto& waypt : landmark.second.waypoints)
        {
            ref_setpoints.ref.emplace_back(waypt);
        }
        ref_setpoints.id = 0;
    }
    bReturn = true;

    if (bReturn)
        updateData(ref_setpoints);

    return bReturn;
}

void CDroneWaypointsFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // TODO
}

// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleMissionPlanner.h"
#include <mutex>
#include <os/CFileUtils.h>
#include <math/CGeometry.h>
#include <csv_reader.h>

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleMissionPlanner(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleMissionPlanner(_params);
}

CVehicleMissionPlanner::CVehicleMissionPlanner(CycDatablockKey key) :
    CCycFilterBase(key)
{
    setFilterType("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE");
	m_OutputDataType = CyC_POINTS;
}

CVehicleMissionPlanner::CVehicleMissionPlanner(ConfigFilterParameters params) : CCycFilterBase(params)
{
    setFilterType("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_POINTS;

    m_csvPath = fs::path(params.sGlobalBasePath) / fs::path(m_CustomParameters.at("reference_path"));
}

CVehicleMissionPlanner::~CVehicleMissionPlanner()
{
    if (m_bIsEnabled)
        disable();
}

bool CVehicleMissionPlanner::enable()
{
    m_bIsEnabled = CFileUtils::FileExist(m_csvPath.c_str());
 
    for (CycInputSource& src : getInputSources())
    {
        if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE") ||
            src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
            m_pFilterVehicleState = src.pCycFilter;
        else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_TERMINAL_VEHICLE_FILTER_TYPE"))
            m_pVehicleTerminalSource = src.pCycFilter;
    }

    if (m_bIsEnabled)
        spdlog::info("Filter [{}-{}]: {}::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
    else
        spdlog::error("Filter [{}-{}]: {}::enable() Error: mission planning file not found.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());

    return m_bIsEnabled;
}

bool CVehicleMissionPlanner::disable()
{
    if (isRunning())
        stop();

    m_bIsEnabled = false;
    return true;
}

CycLandmark getClosestTraj(const CycLandmarks& _landmarks, const CPose& _dst)
{
    float fMinEd = std::numeric_limits<float>::max();
    CycLandmark ref_traj;
    Eigen::Vector2f destination = _dst.translation_3x1().head<2>();
    for (const auto& traj : _landmarks)
    {
        Eigen::Vector2f landmark = traj.second.pose.translation_3x1().head<2>();

        float fEd = CGeometry::euclidean_dist(destination, landmark);
        if (fEd < fMinEd)
        {
            ref_traj = traj.second;
            fMinEd = fEd;
        }
    }
    return ref_traj;
}

bool CVehicleMissionPlanner::process()
{
    bool bReturn = false;
    std::string path = m_csvPath;

    if (m_pVehicleTerminalSource) // we get the mission from the server
    {
        CycTerminalCommand cmd;

        if((m_pVehicleTerminalSource->getTimestampStop() > m_lastTerminalReadTime) && m_pVehicleTerminalSource->getData(cmd))
        {
            m_lastTerminalReadTime = m_pVehicleTerminalSource->getTimestampStop();
            
            // In case a new set of landmarks and waypoints are received
            if (cmd.landmarks.size())
            {
                // Save to dynamic mission (it will be loaded automatically at boot)
                m_WaypointsPlanner.loadWaypoints(cmd.landmarks);
                m_WaypointsPlanner.saveWaypoints(cmd.landmarks, m_csvPath);

                if (cmd.landmarks.find(0) != cmd.landmarks.end())
                {
                    m_MissionConverter.landmark2mission(cmd.landmarks.at(0), m_ReferencePathPoints);
                    cmd.destination = cmd.landmarks.at(0).pose; // if landmarks are received, then default to the first mission
                    bReturn = true;
                }

                cmd.landmarks.clear(); // clear so we don't load the same landmarks multiple times.
                m_pVehicleTerminalSource->updateData(cmd);
            }

            const CycLandmarks& landmarks = m_WaypointsPlanner.getLandmarks();

            // Check the trajectory having an end position closest to the received destination
            CycLandmark ref_traj;
            
            if (cmd.destination.translation_3x1().x() == 0.f && cmd.destination.translation_3x1().y() == 0.f)
            {
                if (landmarks.find(0) != landmarks.end())
                    ref_traj = landmarks.at(0);
            }
            else
            {
                ref_traj = getClosestTraj(landmarks, cmd.destination);
            }

            if (ref_traj.waypoints.size() > 0)
            {
                m_MissionConverter.landmark2mission(ref_traj, m_ReferencePathPoints);
                bReturn = true;
            }
        }
    }
    
    if (!m_bRead) // We have a standard trajectory file
    {
        csv::reader csv_reader;
        if (!csv_reader.open(path))
        {
            spdlog::error("Filter [{}-{}]: {}: failed to open csv {}", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name(), path);
            return false;
        }

        m_ReferencePathPoints.clear();

        // Check if GPS file
        auto column_names = csv_reader.get_column_names();

        for (const auto& column_name : column_names)
        {
            const std::string col_name = "state_variable_";

            // If we have ''lat'' in the csv file header, then we have a GPS csv file
            if (column_name.compare("lat") == 0)
            {
                m_bIsGpsFile = true;
                break;
            }
            // If we have variables with "state_variable_", then we have a state file
            else if (column_name.compare(0, col_name.size(), col_name) == 0)
            {
                m_bIsStateFile = true;
                break;
            }
            // If we have "waypoints" in the csv file header, then we have a waypoints csv file
            else if (column_name.compare("waypoints") == 0)
            {
                m_bIsWaypointsFile = true;
                break;
            }
        }

        if (m_bIsGpsFile)
        {
            bReturn = m_MissionConverter.gps2mission(csv_reader, m_ReferencePathPoints);
        }
        else if (m_bIsStateFile)
        {
            bReturn = m_MissionConverter.state2mission(csv_reader, m_ReferencePathPoints);
        }
        else if (m_bIsWaypointsFile)
        {
            if (m_WaypointsPlanner.loadWaypoints(path))
            {
                const CycLandmarks& landmarks = m_WaypointsPlanner.getLandmarks();

                if (landmarks.find(0) != landmarks.end())
                {
                    m_MissionConverter.landmark2mission(landmarks.at(0), m_ReferencePathPoints);
                    bReturn = true;
                }
            }
        }
        else
        {
            // not implemented, unknown file type
        }

        m_bRead = true;
    }

    if (bReturn)
        updateData(m_ReferencePathPoints);

    return bReturn;
}

void CVehicleMissionPlanner::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{}

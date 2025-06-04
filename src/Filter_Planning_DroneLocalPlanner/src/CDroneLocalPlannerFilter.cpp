// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CDroneLocalPlannerFilter.h"
#include "setg/CSETG.h"

EPlannerType string_to_planner_type(const std::string& arg)
{
    if (arg == "SETG")
        return EPlannerType::SETG;

    return EPlannerType::UNKNOWN;
}

CDroneLocalPlannerFilter::CDroneLocalPlannerFilter(CycDatablockKey key) :
    CCycFilterBase(key)
{
    init();
}

CDroneLocalPlannerFilter::CDroneLocalPlannerFilter(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
    init();
}

void CDroneLocalPlannerFilter::init()
{
    // Assign the output data type
    setFilterType("CyC_DRONE_LOCAL_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_LANDMARKS;
}

CDroneLocalPlannerFilter::~CDroneLocalPlannerFilter()
{
    if (m_bIsEnabled)
        disable();
}

void getCustomParameter(CyC_INT& value, const std::string& param)
{
    if (param.empty())
    {
        return;
    }

    char* ptr_end = nullptr;
    const CyC_INT tmpVal = std::strtol(param.c_str(), &ptr_end, 10);
    if (ptr_end != param.c_str())
    {
        value = tmpVal;
    }
    else
    {
        spdlog::warn("CDroneLocalPlannerFilter: Failed to convert parameter to int: {}", param);
    }
}

void getCustomParameter(float& value, const std::string& param)
{
    if (param.empty())
    {
        return;
    }

    char* ptr_end = nullptr;
    const float tmpVal = std::strtof(param.c_str(), &ptr_end);
    if (ptr_end != param.c_str())
    {
        value = tmpVal;
    }
    else
    {
        spdlog::warn("CDroneLocalPlannerFilter: Failed to convert parameter to float: {}", param);
    }
}

void getCustomParameter(bool& value, const std::string& param)
{
    value = (param == "true");
}

bool CDroneLocalPlannerFilter::enable()
{
    for (const auto& source : getInputSources())
    {
        if (source.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_DRONE_WAYPOINTS_PLANNER_FILTER_TYPE"))
        {
            m_pInputDroneWaypoints = source.pCycFilter;
        }
        else if (source.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_SENSOR_FUSION_MODEL_FILTER_TYPE") ||
            source.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_MAP_FILTER_TYPE"))
        {
            m_pMapFilter = source.pCycFilter;
        }
        else
        {
            log_error("CDroneLocalPlannerFilter: Input filter {}-{} cannot be used as input.",
                source.pCycFilter->getFilterKey().nCoreID,
                source.pCycFilter->getFilterKey().nFilterID);
        }
    }

    // Check if the waypoints_map file exists
    if (m_pInputDroneWaypoints == nullptr)
    {
        log_error("No available waypoints input.");
        return false;
    }

    m_plannerType = string_to_planner_type(m_CustomParameters.at("type"));
    if (m_plannerType == EPlannerType::UNKNOWN)
    {
        log_error("Unknown planner type.Available options are : SETG, CMNST.");
        return false;
    }

    log_info("CDroneLocalPlannerFilter::enable() successful");

    m_bIsEnabled = true;
    return m_bIsEnabled;
}

bool CDroneLocalPlannerFilter::disable()
{
    if (isRunning())
        stop();

    m_bIsEnabled = false;
    return true;
}

bool CDroneLocalPlannerFilter::process()
{
    bool bReturn = false;
    CycReferenceSetPoints ref_setpoints;
    std::vector<Eigen::VectorXf> obstacles;

    // Get environment map (obstacles)
    if (m_pMapFilter != nullptr)
    {
        CycEnvironment environmentSnapshot(1.F);
        if (m_pMapFilter->getData(environmentSnapshot))
        {
            for (const auto obj : environmentSnapshot.objects)
            {
                obstacles.emplace_back(util::makeVector(
                    obj.origin.translation_3x1().x(),
                    obj.origin.translation_3x1().y(),
                    obj.origin.translation_3x1().z(),
                    obj.width, obj.height, obj.depth));
            }
        }
    }

    // Get mission from waypoints planner
    const auto droneWaypointsTs = m_pInputDroneWaypoints->getTimestampStop();
    if ((droneWaypointsTs > m_prevTsDroneWaypoints) && m_pInputDroneWaypoints->getData(ref_setpoints))
    {
        if (m_plannerType == EPlannerType::SETG)
        {
            CSETG algo;
            ref_setpoints.ref = algo.computePath(ref_setpoints.ref, obstacles, 0.2F);

            // estimate yaw
            if (ref_setpoints.ref.size() >= 2)
            {
                ref_setpoints.ref[0].conservativeResize(4);
                ref_setpoints.ref[0][3] = atan2(ref_setpoints.ref[1].y() - ref_setpoints.ref[0].y(), ref_setpoints.ref[1].x() - ref_setpoints.ref[0].x());
                for (size_t i = 1; i < ref_setpoints.ref.size(); ++i)
                {
                    ref_setpoints.ref[i].conservativeResize(4);
                    ref_setpoints.ref[i][3] = atan2(ref_setpoints.ref[i].y() - ref_setpoints.ref[i-1].y(), ref_setpoints.ref[i].x() - ref_setpoints.ref[i-1].x());
                }
            }

            m_lastTrajectoryId = ref_setpoints.id;
            bReturn = true;
        }

        m_prevTsDroneWaypoints = droneWaypointsTs;
    }

    if (bReturn)
        updateData(ref_setpoints);

    return bReturn;
}

void CDroneLocalPlannerFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // TODO
}

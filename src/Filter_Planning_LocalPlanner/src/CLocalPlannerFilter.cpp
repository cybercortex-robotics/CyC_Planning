// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CLocalPlannerFilter.h"
#include "planners/CPlannerCMNST.h"

CLocalPlannerFilter::CLocalPlannerFilter(CycDatablockKey key) :
    CCycFilterBase(key),
    m_pEnvironmentModel(1.F)
{
    setFilterType("CyC_LOCAL_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_REFERENCE_SETPOINTS;
}

CLocalPlannerFilter::CLocalPlannerFilter(const ConfigFilterParameters& params)
    : CCycFilterBase(params),
    m_pEnvironmentModel(1.F)
{
    setFilterType("CyC_LOCAL_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_REFERENCE_SETPOINTS;
}

CLocalPlannerFilter::~CLocalPlannerFilter()
{
    if (m_bIsEnabled)
        disable();
}

CLocalPlannerFilter::PlannerType CLocalPlannerFilter::str2planner(const std::string& str_type)
{
    if (str_type == "CMNST")
        return PlannerType::CMNST;

    return PlannerType::None;
}

bool CLocalPlannerFilter::enable()
{
    if (!isNetworkFilter() && !isReplayFilter())
    {
        for (const CycInputSource& src : getInputSources())
        {
            if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE"))
            {
                m_InputSourceMissionPlanner = src;
                log_info("Mission planner filter found.");
            }
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_DRONE_SIMULATION_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_DRONE_STATE_ESTIMATION_FILTER_TYPE"))
            {
                m_InputSourceRobotStateMeasurement = src;
                log_info("State filter found.");
            }
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_SENSOR_FUSION_MODEL_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_MAP_FILTER_TYPE"))
            {
                m_InputSourceEnvironmentModel = src;
                log_info("Sensor fusion filter found.");
            }
            else
            {
                // Nothing to do
            }
        }

        if (m_InputSourceMissionPlanner.pCycFilter == nullptr)
        {
            log_error("No mission planning filter given as input.");
            return false;
        }

        if (m_InputSourceRobotStateMeasurement.pCycFilter == nullptr)
        {
            log_error("No state measurement filter given as input or robot not supported yet.");
            return false;
        }

        if (m_InputSourceEnvironmentModel.pCycFilter == nullptr)
        {
            log_warn("No sensor fusion filter given as input.");
        }

        // Get local planner type
        m_LocalPlannerType = str2planner(m_CustomParameters["type"]);
        if (m_LocalPlannerType == PlannerType::None)
        {
            log_error("Type of planner is not known: {}", m_CustomParameters["type"]);
            return false;
        }

        m_plannerCMNST = std::make_unique<CPlannerCMNST>();
        m_plannerCMNST->loadConfiguration(m_CustomParameters);

        // Set classes that are not obstacles
        std::vector<std::string> tokens;
        CStringUtils::splitstring(m_CustomParameters["traversible_classes"], ",", tokens);

        for (const auto& token : tokens)
        {
            if (CStringUtils::is_positive_int(token))
            {
                m_TraversableClassIDs.emplace_back(std::stoi(token));
            }
            else
            {
                log_warn("Could not convert '{}' to traversible class id.", token);
            }
        }
    }

    m_bIsEnabled = true;
    log_info("enable() successful");

    return true;
}

bool CLocalPlannerFilter::disable()
{
    if (isRunning())
        stop();

    m_bIsEnabled = false;
    return true;
}

bool CLocalPlannerFilter::process()
{
    bool bReturn = false;

    // Get the global reference path from the mission planner
    if (m_MissionPlan.empty() && m_InputSourceMissionPlanner.pCycFilter != nullptr)
    {
        while (!m_InputSourceMissionPlanner.getData(m_MissionPlan))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    const bool bReadEnvModel = m_InputSourceEnvironmentModel.getData(m_pEnvironmentModel);
    const bool bReadState = m_InputSourceRobotStateMeasurement.getData(m_VehicleState);

    // Setpoints (local reference path)
    CycReferenceSetPoints reference_setpoints;

    if (bReadEnvModel && bReadState)
    {
        // do something with env and state
    }

    if (m_LocalPlannerType == PlannerType::CMNST && bReadState)
    {
        // CMNST does not require state or env model, yet
        std::vector<Eigen::Vector3f> obstacles;
        reference_setpoints = m_plannerCMNST->compute(obstacles, m_MissionPlan, m_VehicleState, 1.F);

        bReturn = !reference_setpoints.ref.empty();
    }

    if (bReturn)
        updateData(reference_setpoints);

    return bReturn;
}

void CLocalPlannerFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // Set the processing flag
    m_bIsProcessing = true;

    csv::reader::row row;
    row.parse_line(datastream_entry, ',');

    enum { TS_STOP, SAMPLING_TIME, FRAME_ID, NUM };
    if (row.size() != NUM)
    {
        log_error("Wrong number of columns. {} provided, but expected {}.", row.size(), NUM);
        return;
    }

    enum { REF_FRAME_ID, REF_NUM };
    if (m_first_read)
    {
        m_first_read = false;
        if (!m_reference_reader.open(db_root_path + "framebased_data_descriptor.csv"))
        {
            log_error("Could not open {}", db_root_path + "framebased_data_descriptor.csv");
        }
        else if (m_reference_reader.get_column_names().size() < REF_NUM)
        {
            log_error("Wrong number of columns. Expected at least {}, got {}.", REF_NUM + 1, m_reference_reader.get_column_names().size());
        }
        else
        {
            m_last_read_success = m_reference_reader.next_row();
            if (!m_last_read_success)
            {
                log_warn("Framebased data descriptor is empty.");
            }
        }
    }

    CycReferenceSetPoints reference_setpoints;
    reference_setpoints.ref.clear();
    if (!m_reference_reader.is_open())
    {
        return;
    }

    if (!m_last_read_success)
    {
        log_warn("Reached end of data stream.");
        return;
    }

    const auto sync_frame_id = row.get<CyC_INT>(FRAME_ID);
    auto ref_frame_id = m_reference_reader.get_row().get<CyC_INT>(REF_FRAME_ID);

    // sync
    while (m_last_read_success && (ref_frame_id < sync_frame_id))
    {
        m_last_read_success = m_reference_reader.next_row();
        if (m_last_read_success)
        {
            ref_frame_id = m_reference_reader.get_row().get<CyC_INT>(REF_FRAME_ID);
        }
    }

    while (m_last_read_success && (ref_frame_id == sync_frame_id))
    {
        const auto& ref_row = m_reference_reader.get_row();
        reference_setpoints.ref.emplace_back(ref_row.size() - 1);
        for (auto i = 0; i < reference_setpoints.ref.back().size(); ++i)
        {
            reference_setpoints.ref.back()[i] = ref_row.get<float>(i+1);
        }

        m_last_read_success = m_reference_reader.next_row();
        if (m_last_read_success)
        {
            ref_frame_id = ref_row.get<CyC_INT>(REF_FRAME_ID);
        }
    }

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;

    updateData(reference_setpoints, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

    // Unset the processing flag
    m_bIsProcessing = false;
}


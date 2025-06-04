// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleLocalPlannerFilter.h"
#include <mutex>
#include <csv_reader.h>

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleLocalPlannerFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleLocalPlannerFilter(_params);
}

CVehicleLocalPlannerFilter::CVehicleLocalPlannerFilter(CycDatablockKey key) :
    CCycFilterBase(key),
    m_pEnvironmentModel(1.F)
{
    setFilterType("CyC_VEHICLE_LOCAL_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_REFERENCE_SETPOINTS;
}

CVehicleLocalPlannerFilter::CVehicleLocalPlannerFilter(const ConfigFilterParameters& params)
    : CCycFilterBase(params),
    m_pEnvironmentModel(1.F)
{
    setFilterType("CyC_VEHICLE_LOCAL_PLANNER_FILTER_TYPE");
    m_OutputDataType = CyC_REFERENCE_SETPOINTS;

    // Get local planner type
    if (!m_CustomParameters["LocalPlanner"].empty())
    {
        m_LocalPlannerType = str2VehiclePlannerType(m_CustomParameters["LocalPlanner"]);
    }
    else
    {
        spdlog::warn("Filter [{}-{}]: LocalPlanner variable not set in the configuration file. Defaulting to blind planner", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_LocalPlannerType = VehiclePlanner_BLIND;
    }

    if (m_LocalPlannerType != VehiclePlanner_BLIND && m_LocalPlannerType != VehiclePlanner_ASTAR && 
        m_LocalPlannerType != VehiclePlanner_DWA && m_LocalPlannerType != VehiclePlanner_LATTICE)
    {
        spdlog::warn("Filter [{}-{}]: LocalPlanner value \"{}\" incorrect. Defaulting to blind planner.", m_CustomParameters["LocalPlanner"], getFilterKey().nCoreID, getFilterKey().nFilterID);
        spdlog::warn("Filter [{}-{}]: Available LocalPlanner options are blind, dwa, astar and lattice", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_LocalPlannerType = VehiclePlanner_BLIND;
    }

    // Target speed
    if (!m_CustomParameters["TargetSpeed"].empty())
        m_fTargetSpeed = std::stof(m_CustomParameters["TargetSpeed"]);

    // Lookahead distance
    if (!m_CustomParameters["LookaheadDistance"].empty())
    {
        m_fLookaheadDistance = std::stof(m_CustomParameters["LookaheadDistance"]);
    }
    else
    {
        m_fLookaheadDistance = 2.F;
        spdlog::warn("Filter [{}-{}]: LocalPlanner: Lookahead distance not set in configuration file. Defaulting to {}", getFilterKey().nCoreID, getFilterKey().nFilterID, m_fLookaheadDistance);
    }

    // Set the ground class IDs used for projecting the segmented ground into the octree
    if (!m_CustomParameters["ground_class_ids"].empty())
    {
        std::vector<std::string> tokens;
        CStringUtils::splitstring(m_CustomParameters["ground_class_ids"], ",", tokens);

        for (const auto& token : tokens)
            if (CStringUtils::is_positive_int(token))
                m_TraversableClassIDs.emplace_back(std::stoi(token));
            else
                spdlog::warn("CSensorFusionFilter::init(): Could not convert '{}' to ground class id.", token);
    }
}
;
CVehicleLocalPlannerFilter::~CVehicleLocalPlannerFilter()
{
    if (m_bIsEnabled)
        disable();
}

bool CVehicleLocalPlannerFilter::enable()
{
    bool bFoundMissionPlanningFilter = false;
    bool bFoundStateMeasurementFilter = false;

    if (!isNetworkFilter() && !isReplayFilter())
    {
        for (CycInputSource& src : getInputSources())
        {
            if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE"))
            {
                m_pInputSourceMissionPlanner = &src;
                bFoundMissionPlanningFilter = true;
            }
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE"))
            {
                m_pInputSourceVehicleStateMeasurement = &src;
                bFoundStateMeasurementFilter = true;
            }
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
            {
                m_pInputSourceVehicleStateMeasurement = &src;
                bFoundStateMeasurementFilter = true;
                spdlog::info("Filter [{}-{}]: CVehicleLocalPlannerFilter: sensor fusion filter detected.", getFilterKey().nCoreID, getFilterKey().nFilterID);
            }
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_SENSOR_FUSION_MODEL_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_MAP_FILTER_TYPE"))
            {
                m_bEnvironmentModelFound = true;
                m_pInputSourceEnvironmentModel = &src;
                spdlog::info("Filter [{}-{}]: CVehicleLocalPlannerFilter: map filter filter detected.", getFilterKey().nCoreID, getFilterKey().nFilterID);
            }
        }

        if (!bFoundMissionPlanningFilter)
        {
            spdlog::error("Filter [{}-{}]: CVehicleLocalPlannerFilter: No mission planning filter given as input. Disabling the CVehicleLocalPlannerFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID);
            return false;
        }

        if (!bFoundStateMeasurementFilter)
        {
            spdlog::error("Filter [{}-{}]: CVehicleLocalPlannerFilter: No state measurement filter given as input. Disabling the CVehicleLocalPlannerFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID);
            return false;
        }

        if (!m_bEnvironmentModelFound)
        {
            spdlog::info("Filter [{}-{}]: CVehicleLocalPlannerFilter: no sensor fusion filter detected.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        }

        float goal_distance = 1.f;
        if (!m_CustomParameters["goal_distance"].empty())
            goal_distance = std::stof(m_CustomParameters["goal_distance"]);
        else
            spdlog::warn("Filter [{}-{}]: CVehicleLocalPlannerFilter: goal_distance not found inside control filter parameters. Defaulting to: {}", getFilterKey().nCoreID, getFilterKey().nFilterID, 1);

        // Set the traversable class IDs used for projecting the segmented traversable into the octree
        if (!m_CustomParameters["traversable_class_ids"].empty())
        {
            std::vector<std::string> tokens;
            CStringUtils::splitstring(m_CustomParameters["traversable_class_ids"], ",", tokens);

            for (const auto& token : tokens)
                if (CStringUtils::is_positive_int(token))
                    m_TraversableClassIDs.emplace_back(std::stoi(token));
                else
                    spdlog::warn("CSensorFusionFilter::init(): Could not convert '{}' to traversable class id.", token);
        }

        // Initilize local planner
        if (!m_CustomParameters["vehicle_model"].empty())
        {
            std::string strVehicleModelFile = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("vehicle_model"));

            if (!fs::exists(strVehicleModelFile))
            {
                spdlog::error("Filter [{}-{}]: CVehicleLocalPlannerFilter: No vehicle model file defined. CVehicleLocalPlannerFilter disabled.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                return false;
            }

            switch (m_LocalPlannerType)
            {
                case VehiclePlanner_BLIND:
                    m_pLocalPlannerBlind = std::make_unique<CBlindPlanner>(strVehicleModelFile, m_fLookaheadDistance, float(this->getDt() * MSEC2SEC));
                    break;
                case VehiclePlanner_DWA:
                    m_pLocalPlannerDwa = std::make_unique<CDwa>(float(this->getDt() * MSEC2SEC), strVehicleModelFile, m_fLookaheadDistance, goal_distance, m_TraversableClassIDs);
                    break;
                case VehiclePlanner_ASTAR:
                    m_pLocalPlannerAstar = std::make_unique<CAstarPlanner>(strVehicleModelFile, m_fLookaheadDistance, float(this->getDt() * MSEC2SEC));
                    break;
                case VehiclePlanner_LATTICE:
                    m_pLocalPlannerLattice = std::make_unique<CLatticePlanner>(strVehicleModelFile, m_fLookaheadDistance, float(this->getDt() * MSEC2SEC));
                    break;
                default:
                    break;
            }
        }
    }

    m_bIsEnabled = true;
    spdlog::info("Filter [{}-{}]: CVehicleLocalPlannerFilter::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);

    return true;
}

bool CVehicleLocalPlannerFilter::disable()
{
    if (isRunning())
        stop();

    m_bIsEnabled = false;
    return true;
}

bool CVehicleLocalPlannerFilter::process()
{
    bool bReturn(false);

    // Get the global reference path from the vehicle mission planner
    bool missionRead = false;
    while (!missionRead)
    {
        if (m_pInputSourceMissionPlanner->pCycFilter->getData(m_MissionPlan) &&
            m_pInputSourceVehicleStateMeasurement->pCycFilter->getData(m_VehicleState))
        {
            missionRead = true;

            if (m_pLocalPlannerDwa != nullptr)
                m_pLocalPlannerDwa->setMissionPath(m_MissionPlan, m_VehicleState);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Setpoints (local reference path)
    CycReferenceSetPoints reference_setpoints;

    const auto bReadEnvModel = (!m_bEnvironmentModelFound) ||
        (
        //(m_pInputSourceEnvironmentModel->pCycFilter->getTimestampStop() > m_lastReadTSEnvModel) &&
        (m_pInputSourceEnvironmentModel->pCycFilter->getData(m_pEnvironmentModel)));

    const auto bReadState = //(m_pInputSourceVehicleStateMeasurement->pCycFilter->getTimestampStop() > m_lastReadTSState) &&
        (m_pInputSourceVehicleStateMeasurement->pCycFilter->getData(m_VehicleState));
    
    if (bReadEnvModel && bReadState)
    {
        // Update sensor fusion timestamp
        if (m_bEnvironmentModelFound)
        {
            m_lastReadTSEnvModel = m_pInputSourceEnvironmentModel->pCycFilter->getTimestampStop();
        }

        // Update state timestamp
        m_lastReadTSState = m_pInputSourceVehicleStateMeasurement->pCycFilter->getTimestampStop();
        
        // Calculate the local reference path based only on the global mission path
        if (m_LocalPlannerType == VehiclePlanner_BLIND)
        {
            reference_setpoints = m_pLocalPlannerBlind->mission2reference(m_MissionPlan, m_VehicleState, m_fLookaheadDistance, m_fTargetSpeed);
        }
        else if (m_bEnvironmentModelFound)
        {
            if (m_LocalPlannerType == VehiclePlanner_DWA)
                reference_setpoints = m_pLocalPlannerDwa->dwaPlan(m_VehicleState, m_pEnvironmentModel);
            else if (m_LocalPlannerType == VehiclePlanner_ASTAR)
                reference_setpoints = m_pLocalPlannerAstar->perception2referenceAStar(m_MissionPlan, *m_pEnvironmentModel.pOccupancyModel, m_VehicleState);
            else if (m_LocalPlannerType == VehiclePlanner_LATTICE)
                reference_setpoints = m_pLocalPlannerLattice->perception2referenceLP(m_MissionPlan, *m_pEnvironmentModel.pOccupancyModel, m_VehicleState);
        }

        bReturn = true;
    }

    if (bReturn)
        updateData(reference_setpoints);

    return bReturn;
}

void CVehicleLocalPlannerFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // Set the processing flag
    m_bIsProcessing = true;

    csv::reader::row row;
    row.parse_line(datastream_entry, ',');

    enum { TS_STOP, SAMPLING_TIME, FRAME_ID, NUM };
    if (row.size() != NUM)
    {
        spdlog::error("CVehicleLocalPlannerFilter: Wrong number of columns. {} provided, but expected {}.", row.size(), NUM);
        return;
    }

    enum { REF_FRAME_ID, REF_NUM };
    if (m_first_read)
    {
        m_first_read = false;
        if (!m_reference_reader.open(db_root_path + "framebased_data_descriptor.csv"))
        {
            spdlog::error("Filter[{}-{}]: Could not open {}", getFilterKey().nCoreID, getFilterKey().nFilterID, db_root_path + "framebased_data_descriptor.csv");
        }
        else if (m_reference_reader.get_column_names().size() < REF_NUM)
        {
            spdlog::error("CVehicleLocalPlannerFilter: Wrong number of columns. Expected at least {}, got {}.", REF_NUM + 1, m_reference_reader.get_column_names().size());
        }
        else
        {
            m_last_read_success = m_reference_reader.next_row();
            if (!m_last_read_success)
            {
                spdlog::warn("Filter[{}-{}]: Framebased data descriptor is empty.", getFilterKey().nCoreID, getFilterKey().nFilterID);
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
        spdlog::warn("Filter[{}-{}]: Reached end of data stream.", getFilterKey().nCoreID, getFilterKey().nFilterID);
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

CVehicleLocalPlannerFilter::VehiclePlannerType CVehicleLocalPlannerFilter::str2VehiclePlannerType(const std::string& str_type)
{
    if (str_type.compare("blind") == 0)
		return VehiclePlanner_BLIND;
	else if (str_type.compare("astar") == 0)
		return VehiclePlanner_ASTAR;
    else if (str_type.compare("dwa") == 0)
		return VehiclePlanner_DWA;
    else if (str_type.compare("lattice") == 0)
		return VehiclePlanner_LATTICE;

	return VehiclePlanner_BLIND;
}

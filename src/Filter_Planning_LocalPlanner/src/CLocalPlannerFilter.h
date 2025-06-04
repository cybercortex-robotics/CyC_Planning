// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CyC_FILTER_LOCALPLANNER_H
#define CyC_FILTER_LOCALPLANNER_H

#include <CyC_TYPES.h>
#include <CCycFilterBase.h>
#include <csv_reader.h>

class CPlannerCMNST;

class CLocalPlannerFilter : public CCycFilterBase
{
private:
    enum class PlannerType
    {
        None = 0,
        CMNST = 1
    };

public:
	explicit CLocalPlannerFilter(CycDatablockKey key);
    explicit CLocalPlannerFilter(const ConfigFilterParameters& params);
    ~CLocalPlannerFilter() override;

    bool enable() override;
    bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

    static PlannerType str2planner(const std::string& str_type);

private:
    // Used in loadFromDatastream
    bool m_first_read = true;
    bool m_last_read_success = true;
    csv::reader m_reference_reader;

    // Input source filters
    CycInputSource m_InputSourceMissionPlanner;
    CycInputSource m_InputSourceRobotStateMeasurement;
    CycInputSource m_InputSourceEnvironmentModel;

    // Global mission plan
    std::vector<Eigen::Vector4f> m_MissionPlan;

    // Local path planner
    PlannerType m_LocalPlannerType = PlannerType::None;
    std::unique_ptr<CPlannerCMNST> m_plannerCMNST;
    
    // Environment model
    CycEnvironment          m_pEnvironmentModel;
    std::vector<CyC_INT>    m_TraversableClassIDs;

    // Vehicle state measurement
    CycState m_VehicleState;
};

#endif // CyC_FILTER_LOCALPLANNER_H

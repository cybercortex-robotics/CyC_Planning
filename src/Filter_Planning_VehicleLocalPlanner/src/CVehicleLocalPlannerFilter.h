// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLELOCALPLANNER_H_
#define CVEHICLELOCALPLANNER_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include <csv_reader.h>
#include "CBlindPlanner.h"
#include "plan/CDwa.h"
#include "CAstarPlanner.h"
#include "CLatticePlanner.h"

class CVehicleLocalPlannerFilter : public CCycFilterBase
{
private:
    enum VehiclePlannerType
    {
        VehiclePlanner_BLIND = 0,
        VehiclePlanner_ASTAR = 1,
        VehiclePlanner_DWA = 2,
        VehiclePlanner_LATTICE = 3,
    };

public:
	explicit CVehicleLocalPlannerFilter(CycDatablockKey key);
    explicit CVehicleLocalPlannerFilter(const ConfigFilterParameters& params);
    ~CVehicleLocalPlannerFilter() override;

    bool enable() override;
    bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;
    
    VehiclePlannerType str2VehiclePlannerType(const std::string& str_type);

private:
    bool        m_first_read = true;
    bool        m_last_read_success = false;
    csv::reader m_reference_reader;

    // Input source filters
    bool            m_bEnvironmentModelFound = false;
    CycInputSource* m_pInputSourceMissionPlanner;
    CycInputSource* m_pInputSourceVehicleStateMeasurement;
    CycInputSource* m_pInputSourceEnvironmentModel;

    // Global mission plan
    std::vector<Eigen::Vector4f>        m_MissionPlan;

    // Local path planner
    VehiclePlannerType                  m_LocalPlannerType;
    std::unique_ptr<CBlindPlanner>      m_pLocalPlannerBlind = nullptr;
    std::unique_ptr<CDwa>               m_pLocalPlannerDwa = nullptr;
    std::unique_ptr<CAstarPlanner>      m_pLocalPlannerAstar = nullptr;
    std::unique_ptr<CLatticePlanner>    m_pLocalPlannerLattice = nullptr;
    
    // Environment model
    CycEnvironment          m_pEnvironmentModel;
    std::vector<CyC_INT>    m_TraversableClassIDs;

    // Vehicle state measurement
    CycState m_VehicleState;

    // Vehicle model
    std::unique_ptr<VehicleModel>   m_pVehicleModel;
    float                           m_fLookaheadDistance = 5.f;
    float                           m_fTargetSpeed = 5.f;

    CyC_TIME_UNIT m_lastReadTSMissionPlan = 0;
    CyC_TIME_UNIT m_lastReadTSEnvModel = 0;
    CyC_TIME_UNIT m_lastReadTSState = 0;
};

#endif /* CVEHICLELOCALPLANNER_H_ */

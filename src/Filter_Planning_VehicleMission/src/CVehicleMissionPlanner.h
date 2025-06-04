// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLEMISSIONPLANNER_H_
#define CVEHICLEMISSIONPLANNER_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "CMissionConverter.h"
#include "env/CGeolocation.h"
#include "plan/CWaypointsPlanner.h"

class CVehicleMissionPlanner : public CCycFilterBase
{
public:
	explicit CVehicleMissionPlanner(CycDatablockKey key);
    explicit CVehicleMissionPlanner(ConfigFilterParameters params);
    ~CVehicleMissionPlanner() override;

    bool enable() override;
    bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& _b_root_path) override;

private:
    std::string m_csvPath;
    CyC_INT     m_PathId = 1;
    bool        m_bPath_2 = false;
    bool        m_bRead = false;
    bool        m_bIsLocalPath = false;

    bool        m_bIsGpsFile = false;
    bool        m_bIsStateFile = false;
    bool        m_bIsWaypointsFile = false;
    
    CWaypointsPlanner               m_WaypointsPlanner;
    CMissionConverter               m_MissionConverter;
    std::vector<Eigen::Vector4f>    m_ReferencePathPoints;

    CCycFilterBase* m_pVehicleTerminalSource = nullptr;
    CCycFilterBase* m_pFilterVehicleState = nullptr;

    CyC_TIME_UNIT m_lastTerminalReadTime = 0;
};

#endif /* CVEHICLEMISSIONPLANNER_H_ */

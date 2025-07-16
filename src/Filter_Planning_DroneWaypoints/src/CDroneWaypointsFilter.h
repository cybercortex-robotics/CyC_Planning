// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDroneWaypointsFilter_H_
#define CDroneWaypointsFilter_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "plan/CWaypointsPlanner.h"

class CDroneWaypointsFilter : public CCycFilterBase
{
public:
    explicit CDroneWaypointsFilter(CycDatablockKey key);
    explicit CDroneWaypointsFilter(const ConfigFilterParameters& params);
    ~CDroneWaypointsFilter() override;

    bool enable() override;
    bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

private:
    CCycFilterBase*     m_pInputDroneTerminal = nullptr;
    CCycFilterBase*     m_pInputArucoDetector = nullptr;
    CyC_TIME_UNIT       m_prevTsDroneTerminal = 0;
    CyC_TIME_UNIT       m_prevTsArucoDetection = 0;
    CWaypointsPlanner   m_WaypointsPlanner;
};

#endif /* CDroneWaypointsFilter_H_ */

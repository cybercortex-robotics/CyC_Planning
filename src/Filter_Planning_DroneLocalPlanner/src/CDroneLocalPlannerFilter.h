// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CDroneLocalPlannerFilter_H_
#define CDroneLocalPlannerFilter_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"

enum class EPlannerType
{
    UNKNOWN,
    SETG
};

class CDroneLocalPlannerFilter : public CCycFilterBase
{
public:
    explicit CDroneLocalPlannerFilter(CycDatablockKey key);
    explicit CDroneLocalPlannerFilter(const ConfigFilterParameters& params);
    ~CDroneLocalPlannerFilter() override;

    bool enable() override;
    bool disable() override;

private:
    void        init();
    bool	process() override;
    void        loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

private:
    CCycFilterBase* m_pInputDroneWaypoints = nullptr;
    CyC_TIME_UNIT   m_prevTsDroneWaypoints = 0;

    CCycFilterBase* m_pMapFilter = nullptr;
    CyC_TIME_UNIT   m_prevTsMap = 0;

    CyC_INT         m_lastTrajectoryId = -1;
    EPlannerType    m_plannerType = EPlannerType::UNKNOWN;
};

#endif /* CDroneLocalPlannerFilter_H_ */

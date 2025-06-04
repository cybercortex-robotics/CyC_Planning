// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef _CBLIND_PLANNER_H_
#define _CBLIND_PLANNER_H_

#include "CyC_TYPES.h"
#include "plan/CPlanningUtils.h"
#include "control/CModelVehicle.h"

class CBlindPlanner
{
public:
    CBlindPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt);
	~CBlindPlanner();

    /**
     * \brief Computes reference setpoints based on the position of the vehicle relative to the global mission path.
     *        The calculated setpoints span across <<length>> meters
     *
     * \param _mission_path     Global mission path
     * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
     * \param length            Desired length (in m)
     **/
    CycReferenceSetPoints mission2reference(const std::vector<Eigen::Vector4f>& _mission_path, 
        const CycState& _vehicle_state, 
        const float _length,
        const float _target_speed = 0);
    
private:
    size_t m_PreviousTrajectoryPointIndex;
};
#endif //_CBLIND_PLANNER_H_

// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CBlindPlanner.h"
#include "plan/CPlanningUtils.h"


CBlindPlanner::CBlindPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt)
{
    m_PreviousTrajectoryPointIndex = 0;
}

CBlindPlanner::~CBlindPlanner()
{}

CycReferenceSetPoints CBlindPlanner::mission2reference(const std::vector<Eigen::Vector4f>& _mission_path, 
    const CycState& _vehicle_state, 
    const float _length,
    const float _target_speed)
{
    auto index = CPlanningUtils::findClosestPoint(_mission_path, _vehicle_state, m_PreviousTrajectoryPointIndex);

    m_PreviousTrajectoryPointIndex = index < _mission_path.size() - 1 ? index : 0;
    
    CycReferenceSetPoints local_pts;
    auto dist = 0.f;
    auto lastPt = _mission_path[index];
    while (dist < _length && index < _mission_path.size())
	{
		Eigen::VectorXf _r;
		_r.resize(ModelVehicle_NumOutputs);

        _r[0] = _mission_path[index].x();
        _r[1] = _mission_path[index].y();

        if (_target_speed == 0)
            _r[2] = _mission_path[index][2];
        else
            _r[2] = _target_speed;
        
        _r[3] = _mission_path[index][3];
        _r[4] = 0.f;

		local_pts.ref.push_back(_r);
        dist += sqrtf(powf(lastPt.x() - _mission_path[index].x(), 2) + powf(lastPt.y() - _mission_path[index].y(), 2));
        lastPt = _mission_path[index];
        index++;
    }

    return local_pts;
}

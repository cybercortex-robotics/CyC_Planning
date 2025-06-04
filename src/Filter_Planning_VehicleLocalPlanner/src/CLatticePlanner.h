// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef _CLATTICE_PLANNER_H_
#define _CLATTICE_PLANNER_H_

#include "CyC_TYPES.h"
#include "plan/CPlanningUtils.h"
#include "CTrajectoryGenerator.hpp"

class CLatticePlanner
{
public:
    CLatticePlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt);
	~CLatticePlanner();

    /**
    * \brief Computes reference setpoints based on the environment model and the position of the vehicle relative to the global mission path (using state lattice planning)
    *
    * \param _mission_path     Global mission path
    * \param _octree           Octree environment model
    * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
    * \param _num_traj_points  Number of setpoints (a sequence of setpoints is the path given to the MPC controller)
    **/
    CycReferenceSetPoints perception2referenceLP(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycState &_vehicle_state);

private:

    std::vector<Eigen::Vector4f> getControlPointsInGlobalCoords(size_t _index, const std::vector<Eigen::Vector4f> &_mission_planner);

    std::vector<float> headingCost(std::vector<Eigen::Vector4f> controlPoints, std::vector<std::vector<Eigen::VectorXf>> candidates);

    std::vector<float> obstacleCost(std::vector<Eigen::Vector4f> obstaclePoints, std::vector<std::vector<Eigen::VectorXf>> candidates);

    void discardInvalidTrajectories(std::vector<std::vector<Eigen::VectorXf>> &candidates, Eigen::Vector4f goalPt, float laneWidth);

    bool detectCollision(Eigen::VectorXf state, Eigen::Vector4f obstaclePt);

    Eigen::Vector2f m_gridMapSize = { 20.f, 20.f };
    size_t m_PreviousTrajectoryPointIndex = 0;
    std::unique_ptr<CTrajectoryGenerator> m_pTrajectoryGenerator;
    std::unique_ptr<VehicleModel> m_pVehicleModel;
    float m_LookaheadDistance;
};
#endif //_CLATTICE_PLANNER_H_

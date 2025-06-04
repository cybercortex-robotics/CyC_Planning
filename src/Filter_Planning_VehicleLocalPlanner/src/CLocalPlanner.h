// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef _CLOCAL_PLANNER_H_
#define _CLOCAL_PLANNER_H_

#include "CyC_TYPES.h"
#include "AStar.hpp"
#include "CDynamicWindowApproach.h"
#include "../../ccr_core/CConfigParameters.h"
#include "CTrajectoryGenerator.hpp"
#include "env/CGridMapUtils.h"
#include "env/COcTreeUtils.h"

class CLocalPlanner
{
public:
    CLocalPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt);
	~CLocalPlanner();

    //CycReferenceSetPoints lanes2reference(const std::vector<Eigen::Vector4f>& _mission_path, const CycLanesModel& _lanes_model, const CycStateMeasurement& _vehicle_state, float length);

    /**
     * \brief Computes reference setpoints based on the environment model and the position of the vehicle relative to the global mission path
     *
     * \param _mission_path     Global mission path
     * \param _octree           Octree environment model
     * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
     **/
    //CycReferenceSetPoints perception2referenceAStar(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycStateMeasurement &_vehicle_state);

    /**
    * \brief Computes reference setpoints based on the environment model and the position of the vehicle relative to the global mission path (using DWA planning)
    *
    * \param _mission_path     Global mission path
    * \param _octree           Octree environment model
    * \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
    **/
    //CycReferenceSetPoints perception2referenceDWA(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycStateMeasurement &_vehicle_state);
/**
	* \brief Computes reference setpoints based on the environment model and the position of the vehicle relative to the global mission path (using state lattice planning)
	*
	* \param _mission_path     Global mission path
	* \param _octree           Octree environment model
	* \param _vehicle_state    State of the vehicle (x, y, velocity, yaw)
	* \param _num_traj_points  Number of setpoints (a sequence of setpoints is the path given to the MPC controller)
	**/
	//CycReferenceSetPoints perception2referenceLP(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycStateMeasurement &_vehicle_state);

    /**
     * \brief Computes the local reference path from a gridmap using A*
     *
     * \param _gridmap              Input gridmap
     * \param _destination          Input destination point (path to destionation point solved using A*)
     * \param _local_planner_path   Output local planner path
     * \param _fake_obstacles       Output fake obstacles added to the gridmap
     **/
    //void gridmap2localpath(const std::vector<Eigen::Vector2i> &_local_points, const Eigen::MatrixXi& _gridmap, const Eigen::Vector2i& _destination, std::vector<Eigen::VectorXf>& _local_planner_path, std::vector<Eigen::Vector2f>& _fake_obstacles);

    /**
    * \brief Computes the local reference path from a gridmap using DWA
    *
    * \param _gridmap              Input gridmap
    * \param _destination          Input destination point
    * \param _local_planner_path   Output local planner path
    **/
    
    //void gridmap2localpathDWA(
        // const CcrOcTree& _octree,
        // const std::vector<Eigen::VectorXf>& _destination,
        // const float& _target_speed,
        // std::vector<Eigen::VectorXf>& _local_planner_path);
    
    /**
    * \brief Find the coordinates of the goal point in the A* coordinate system
    *
    * \param _goal_point        Point coordinates of the destination point in vehicle coordinate system
    * \param _vehicle_state     State of the vehicle (x, y, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    //static Eigen::Vector2i getGoalPointInAStarCoords(const Eigen::Vector4f &_goal_point, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size);

    /**
    * \brief Find the coordinates of the point in the planner coordinate system
    *
    * \param _point             Point coordinates of the destination point in vehicle coordinate system
    * \param _vehicle_state     State of the vehicle (x, y, yaw)
    * \param _octree_resolution Input octree resolution (in meters)
    * \param _gridmap_size      Input gridmap size (in meters)
    **/
    //Eigen::VectorXf getPointInPlannerCoords(const Eigen::Vector4f& _point, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size);

	//std::vector<Eigen::Vector2i> getLocalPointsInAStarCoords(const int &_index, const std::vector<Eigen::Vector4f> &_mission_planner, const int &_goal_point_idx, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size);

    /**
    * \brief Update internal vehicle state
    *
    * \param _vehicle_state     State of the vehicle (x, y, v, yaw)
    **/
    
    //void updateVehicleState(const CycStateMeasurement& _state);

    //void setMissionPlan(const std::vector<Eigen::Vector4f>& mission_plan) { m_pDwaGenerator->setGlobalTrajectory(mission_plan); }

private:
    //AStar::Generator                        m_AstarGenerator;
    //std::unique_ptr<CDynamicWindowApproach> m_pDwaGenerator;
    
    //CycStateMeasurement          m_VehicleState;
    //Eigen::Vector2f     m_gridMapSize;
    //size_t              m_PreviousTrajectoryPointIndex;

    /**
    * \brief Find closest point from trajectory to the vehicle current position
    *
    * \param _mission_path              Trajectory vector
    * \param _vehicle_state             Current vehicle state
    **/
    //size_t findClosestPoint(const std::vector<Eigen::Vector4f>& _mission_path, const CycStateMeasurement& _vehicle_state);

	/**
	* \brief Represent the local path as a centripetal Catmull-Rom spline 
	*
	* \param path				Point coordinates of the local path transformed from A* coordinate system to world coordinate system		
	**/
	//std::vector<Eigen::VectorXf> catmullRomChain(std::vector<Eigen::VectorXf> path);

	/**
	* \brief Return the points on the spline given by 4 control points
	* \param new_points			Point coordinates of the spline representation of the local path
	* \param p0					First control point
	* \param p1					Second control point
	* \param p2					Third control point
	* \param p3					Fourth control point
	**/
	//void catmullRomSpline(std::vector<Eigen::VectorXf> &new_points, Eigen::VectorXf p0, Eigen::VectorXf p1, Eigen::VectorXf p2, Eigen::VectorXf p3);

	//std::vector<Eigen::Vector4f> getControlPointsInGlobalCoords(int _index, const std::vector<Eigen::Vector4f> &_mission_planner);

	//std::vector<float> headingCost(std::vector<Eigen::Vector4f> controlPoints, std::vector<std::vector<Eigen::VectorXf>> candidates);

	//std::vector<float> obstacleCost(std::vector<Eigen::Vector4f> obstaclePoints, std::vector<std::vector<Eigen::VectorXf>> candidates);

	//std::vector<int> selectParetoOptimalSolutions(std::vector<float> obstacleCosts, std::vector<float> headingCosts);

	//void discardInvalidTrajectories(std::vector<std::vector<Eigen::VectorXf>> &candidates, Eigen::Vector4f goalPt, float laneWidth);

	//bool detectCollision(Eigen::VectorXf state, Eigen::Vector4f obstaclePt);

};
#endif //_CLOCAL_PLANNER_H_

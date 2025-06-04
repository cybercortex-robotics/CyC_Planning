// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CLocalPlanner.h"

CLocalPlanner::CLocalPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt)
{
    //m_AstarGenerator.setHeuristic(AStar::Heuristic::euclidean);
    //m_AstarGenerator.setHeuristic(AStar::Heuristic::manhattan);
    //m_AstarGenerator.setDiagonalMovement(true);
	//m_gridMapSize = { 20.f, 20.f };
    //m_PreviousTrajectoryPointIndex = 0;

    // Init DWA
    //m_pDwaGenerator = std::make_unique<CDynamicWindowApproach>(_vehicle_model_file, _lookahead_distance, _dt);
}

CLocalPlanner::~CLocalPlanner()
{
}

/*size_t CLocalPlanner::findClosestPoint(const std::vector<Eigen::Vector4f>& _mission_path,
    const CycStateMeasurement& _vehicle_state)
{
    Eigen::MatrixXf mission_mat(2, _mission_path.size());
    for (size_t i = 0; i < _mission_path.size(); ++i)
    {
        mission_mat.col(i) << _mission_path[i].x(), _mission_path[i].y();
    }

    mission_mat.colwise() -= _vehicle_state.y_hat.topRows(2);
    const Eigen::VectorXf distances = mission_mat.colwise().squaredNorm();

    size_t selected_index = 0;
    float min_dist = std::numeric_limits<float>::max();
    auto stop_idx = std::min((size_t)distances.size(), m_PreviousTrajectoryPointIndex + 150);
    for (size_t idx = m_PreviousTrajectoryPointIndex; idx < stop_idx; ++idx)
    {
        if (distances[idx] < min_dist)
        {
            min_dist = distances[idx];
            selected_index = idx;
        }
    }

    return selected_index;
}*/

/*CycReferenceSetPoints CLocalPlanner::perception2referenceDWA(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycStateMeasurement &_vehicle_state)
{
    CycReferenceSetPoints reference_setpoints;

    // Calculate the local reference path using the environment model and A*
    Eigen::MatrixXi gridmap;
    //Eigen::Vector2f gridmap_size{ 12.f, 12.f }; // the distance in meters which we want to see (both sides of the vehicle)
	COcTreeUtils::octree2gridmap(_octree, gridmap, m_gridMapSize);

    const size_t NUM_GOAL_POINTS = 3;
    const float DISTANCE_BETWEEN_POINTS = m_pDwaGenerator->getLookaheadDistance() / NUM_GOAL_POINTS;
    std::vector<Eigen::VectorXf> goal_points;
    goal_points.reserve(NUM_GOAL_POINTS);

    auto index = findClosestPoint(_mission_path, _vehicle_state);
    m_PreviousTrajectoryPointIndex = index < _mission_path.size() - 1 ? index : 0;
    auto distance_between = [](const Eigen::VectorXf& pt1, const Eigen::Vector4f& pt2)
    {
        return sqrtf(powf(pt1[0] - pt2[0], 2) + powf(pt1[1] - pt2[1], 2));
    };

    Eigen::VectorXf last_point = _vehicle_state.y_hat.topRows(2);
    float fTargetSpeed(0.f);
    for (size_t i = 0; i < NUM_GOAL_POINTS; ++i)
    {
        float dist = distance_between(last_point, _mission_path[index]);
        while ((dist < DISTANCE_BETWEEN_POINTS) && ((index + 1) < _mission_path.size()))
        {
            ++index;
            dist = distance_between(last_point, _mission_path[index]);
        }

        last_point << _mission_path[index].x(), _mission_path[index].y();
        goal_points.emplace_back(last_point);

        fTargetSpeed = _mission_path[index][2];
    }
    
    std::vector<Eigen::VectorXf> reference;
    gridmap2localpathDWA(_octree, goal_points, fTargetSpeed, reference);
    reference_setpoints.ref = reference;

    return reference_setpoints;
}*/

/*CycReferenceSetPoints CLocalPlanner::perception2referenceLP(const std::vector<Eigen::Vector4f>& _mission_path, const CcrOcTree & _octree, const CycStateMeasurement & _vehicle_state)
{
	// TBD - TO REMOVE - to be computed based on the planner's output
	//CycReferenceSetPoints reference_setpoints = mission2reference(_mission_path, _vehicle_state.x_hat, _num_traj_points);
	//reference_setpoints.ref.clear();
	// !TBD - TO REMOVE

	CycReferenceSetPoints reference_setpoints;

	// Calculate the local reference path using the environment model and A*
	Eigen::MatrixXi gridmap;
	//Eigen::Vector2f gridmap_size{ 3.f, 3.f };
	COcTreeUtils::octree2gridmap(_octree, gridmap, m_gridMapSize);

	// Find the goal point
	auto closest_ref_pt = std::min_element(_mission_path.begin(), _mission_path.end(),
		[&](const auto& left, const auto& right)
	{
		const float dist_left = sqrtf(powf(left.x() - _vehicle_state.y_hat(0), 2) + powf(left.y() - _vehicle_state.y_hat(1), 2));
		const float dist_right = sqrtf(powf(right.x() - _vehicle_state.y_hat(0), 2) + powf(right.y() - _vehicle_state.y_hat(1), 2));

		return dist_left < dist_right;
	});

	auto index = std::distance(_mission_path.begin(), closest_ref_pt);
	Eigen::Vector2f startPoint;
	Eigen::Vector2f goalPoint;
	startPoint << _vehicle_state.y_hat(0), _vehicle_state.y_hat(1);
	std::vector<Eigen::Vector4f> controlPoints;
	controlPoints = getControlPointsInGlobalCoords(index, _mission_path);
	goalPoint << controlPoints.at(controlPoints.size() - 1)(0), controlPoints.at(controlPoints.size() - 1)(1);

	// Rotation around the Z axis with the yaw angle of the car
	CPose T_globalpath2vehicle(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -_vehicle_state.y_hat(3));
	Eigen::Vector4f beforeTransformStart(_vehicle_state.y_hat(0), _vehicle_state.y_hat(1), 0.0F, 1.f);
	Eigen::Vector4f afterTransformStart(0.0F, 0.0F, 0.0F, 1.f);
	Eigen::Vector4f beforeTransformGoal(goalPoint(0), goalPoint(1), 0.0F, 1.f);
	Eigen::Vector4f afterTransformGoal(0.0F, 0.0F, 0.0F, 1.f);
	afterTransformStart = T_globalpath2vehicle.transform() * beforeTransformStart;
	afterTransformGoal = T_globalpath2vehicle.transform() * beforeTransformGoal;
	
	// Generate candidate trajectories
	float angle = (controlPoints.at(controlPoints.size() - 1)(3) - _vehicle_state.y_hat(3));
	//float angle = 0;
	float laneWidth = 6.0F;
	CTrajectoryGenerator trajectoryGenerator(afterTransformStart(0), afterTransformStart(1), afterTransformGoal(0), afterTransformGoal(1), angle);
	std::vector<std::vector<Eigen::VectorXf>> laneSamples = trajectoryGenerator.laneStateSampling(laneWidth);

	// Rotate again around the Z axis to revert to the world coordinates system
	CPose T_vehicle2globalpath(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, _vehicle_state.y_hat(3));
	beforeTransformStart = Eigen::Vector4f(afterTransformStart(0), afterTransformStart(1), 0.0F, 1.f);
	beforeTransformGoal = Eigen::Vector4f(afterTransformGoal(0), afterTransformGoal(1), 0.0F, 1.f);
	afterTransformStart = T_vehicle2globalpath.transform() * beforeTransformStart;
	afterTransformGoal = T_vehicle2globalpath.transform() * beforeTransformGoal;
	
	for (int i = 0; i < laneSamples.size(); i++)
	{
		for (int j = 0; j < laneSamples[i].size(); j++)
		{
			Eigen::VectorXf currentPt = laneSamples[i][j];
			Eigen::Vector4f beforeTransformPt(currentPt.x(), currentPt.y(), 0.0F, 1.f);
			Eigen::Vector4f afterTransformPt(0.0F, 0.0F, 0.0F, 1.f);
			afterTransformPt = T_vehicle2globalpath.transform() * beforeTransformPt;
			laneSamples[i][j].x() = afterTransformPt(0);
			laneSamples[i][j].y() = afterTransformPt(1);
		}
	}
	
	//T_vehicle2globalpath = CFrames::create_affine_transformation_matrix(0.0F, 0.0F, _vehicle_state.x_hat(3), Eigen::Vector3f(_vehicle_state.x_hat(0), _vehicle_state.x_hat(1), 0.0F));
	T_vehicle2globalpath.update(_vehicle_state.y_hat(0), _vehicle_state.y_hat(1), 0.0F, 0.0F, 0.0F, _vehicle_state.y_hat(3));
	std::vector<Eigen::Vector4f> obstacles;
	for (CcrOcTree::leaf_iterator it = _octree.begin_leafs(); it != _octree.end_leafs(); it++)
	{
		Eigen::Vector4f obstaclePt;
		obstaclePt << it.getX(), it.getY(), it.getZ(), 1.f;
		Eigen::Vector4f worldObstaclePt = T_vehicle2globalpath.transform() * obstaclePt;
		obstacles.push_back(worldObstaclePt);
	}
	//spdlog::info("Nr of obstacles: {}", obstacles.size());
	
	discardInvalidTrajectories(laneSamples, afterTransformGoal, laneWidth);

	//bestTrajIdx = headingCost(controlPoints, laneSamples);
	std::vector<float> headingCosts = headingCost(controlPoints, laneSamples);
	std::vector<float> obstacleCosts = obstacleCost(obstacles, laneSamples);
	std::vector<int> paretoOptimalSolutions = selectParetoOptimalSolutions(obstacleCosts, headingCosts);

	//spdlog::info("========================================");
	float minCost = FLT_MAX;
	int bestIdx = -1;
	for (int i = 0; i < headingCosts.size(); i++)
	{
		float cost = headingCosts[i] + obstacleCosts[i];
		//float cost = headingCosts[i];
		//spdlog::info("Cost {}: {}", i, cost);
		if (cost < minCost)
		{
			bestIdx = i;
			minCost = cost;
		}
	}
	//spdlog::info("BestIdx: {}", bestIdx);
	for (auto solIdx : paretoOptimalSolutions)
	{
		//spdlog::info("Optimal traj: {}", solIdx);
	}
	//spdlog::info("=======================================s=");

	reference_setpoints.ref = laneSamples.at(bestIdx);
	reference_setpoints.ref_samples = laneSamples;
	
	return reference_setpoints;
}*/

/*CycReferenceSetPoints CLocalPlanner::perception2referenceAStar(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycStateMeasurement &_vehicle_state)
{
    CycReferenceSetPoints reference_setpoints;

    // Calculate the local reference path using the environment model and A*
    Eigen::MatrixXi gridmap;
    //Eigen::Vector2f gridmap_size{ 12.f, 12.f };
	COcTreeUtils::octree2gridmap(_octree, gridmap, m_gridMapSize);

    std::vector<Eigen::VectorXf> astar_local_path;
    std::vector<Eigen::Vector2f> _fakeObstacles;
  
    // Find the goal point
    auto index = findClosestPoint(_mission_path, _vehicle_state);
    m_PreviousTrajectoryPointIndex = index < _mission_path.size() - 1 ? index : 0;
    const CyC_UINT offset = 200;
    Eigen::Vector2i dest_point;
	std::vector<Eigen::Vector2i> local_points;
    if (index + offset < _mission_path.size() - 1)
    {
		local_points = getLocalPointsInAStarCoords(index, _mission_path, offset, _vehicle_state, _octree.getResolution(), m_gridMapSize);
		dest_point = local_points.at(local_points.size() - 1);
        //dest_point = getGoalPointInAStarCoords(_mission_path.at(index + offset), _vehicle_state, _octree.getResolution(), gridmap_size);
    }
    else
    {
        dest_point = getGoalPointInAStarCoords(_mission_path.at(_mission_path.size() - 1), _vehicle_state, _octree.getResolution(), m_gridMapSize);
    }

    gridmap2localpath(local_points, gridmap, dest_point, astar_local_path, _fakeObstacles);

    // A* to world transformation matrix
	CPose T_astar2world(_vehicle_state.y_hat(0), _vehicle_state.y_hat(1), 0.f, 0.F, 0.F, 90.F * DEGTORAD + _vehicle_state.y_hat(3));

    // Scale and transform the planned local path from A* coordinates to global coordinates
    for (const auto& pt_astar : astar_local_path)
    {
        // Coordinates of Astar path, in gridmap coordinate system
        CyC_INT x_astar_coord = pt_astar.x();
        CyC_INT y_astar_coord = pt_astar.y();

        // Here we have to reverse the formula from octree2gridmap function
        // Example for row coordinate (x): 
        // -----------------------------------------------------------------------------
        // CyC_UINT scaling_factor = (CyC_UINT)roundf(1. / _octree.getResolution()); = 1 / 0.1F = 10.F
        // CyC_UINT rows = (CyC_UINT)(_size.y() * scaling_factor);
        // CyC_INT eigen_row_index = rows - (CyC_INT)(((it.getCoordinate().x() + _size.y() / 2.F)) * scaling_factor);
        // => eigen_row_index = _size.y() * scaling_factor - ((it.getCoordinate().x() + _size.y() / 2.F)) * scaling_factor)
        // => eigen_row_index = scaling_factor ( _size.y() - (it.getCoordinate().x() + _size.y() / 2.F))
        // => eigen_row_index = scaling_factor ( _size.y() - it.getCoordinate().x() - _size.y() / 2.F)
        // => eigen_row_index = scaling_factor ( _size.y() / 2 - it.getCoordinate().x())
        // => eigen_row_index / scaling_factor = _size.y() / 2 - it.getCoordinate().x()
        // => it.getCoordinate().x() = _size.y() / 2 - eigen_row_index / scaling_factor
        // -----------------------------------------------------------------------------

        // First scale from A* to world dimensions, but still in gridmap coordinate system
        float x_scaled_coord = x_astar_coord * _octree.getResolution(); // eigen_row_index / scaling_factor
        float y_scaled_coord = y_astar_coord * _octree.getResolution(); // eigen_col_index / scaling_factor

        // Then go back to octree coordinates, but reverse the sign of the y axis
        float x_octree_coord = m_gridMapSize(0) / 2 - x_scaled_coord;
        float y_octree_coord = y_scaled_coord - m_gridMapSize(1) / 2;

        // Now convert to world coordinates by translating with the car's position and rotating with the car's angle
        // The translation and rotation are in T_astar2world
        Eigen::Vector4f before_transform(x_octree_coord, y_octree_coord, 0.f, 1.f);
        Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);

        after_transform = T_astar2world.transform() * before_transform;

        Eigen::VectorXf pt_world = Eigen::VectorXf(2);
        pt_world << after_transform.x(), after_transform.y();

        reference_setpoints.ref.push_back(pt_world);
    }
	
	CycReferenceSetPoints spline_reference_setpoints;
	spline_reference_setpoints.ref = catmullRomChain(reference_setpoints.ref);
	
    return spline_reference_setpoints;
}*/



/*void CLocalPlanner::updateVehicleState(const CycStateMeasurement& _state)
{
    m_VehicleState = _state;
}*/


/*void CLocalPlanner::gridmap2localpathDWA(
    const CcrOcTree& _octree,
    const std::vector<Eigen::VectorXf>& _destination,
    const float& _target_speed,
    std::vector<Eigen::VectorXf>& _local_planner_path)
{
    _local_planner_path.clear();
    Eigen::VectorXf start;
    start.resize(4);
    start << m_VehicleState.y_hat(0), m_VehicleState.y_hat(1), m_VehicleState.y_hat(2), m_VehicleState.y_hat(3);


	//auto T_veh2world = CFrames::create_affine_transformation_matrix(0.F, 0.F, m_VehicleState.x_hat(3), Eigen::Vector3f(m_VehicleState.x_hat(0), m_VehicleState.x_hat(1), 0.f));
	CPose T_veh2world(m_VehicleState.y_hat(0), m_VehicleState.y_hat(1), 0.f, 0.F, 0.F, m_VehicleState.y_hat(3));
    
    std::vector<Eigen::VectorXf> obstacles;
    for (CcrOcTree::leaf_iterator it = _octree.begin_leafs(), end = _octree.end_leafs(); it != end; ++it)
    {
        // Point in vehicle coordinates
        Eigen::Vector4f point;
        point << it.getCoordinate().x(),
                 it.getCoordinate().y(),
                 it.getCoordinate().z(),
				 1.f;
        auto world_obs_pos = T_veh2world.transform() * point;
        obstacles.push_back(world_obs_pos);
    }

    m_pDwaGenerator->dwaControl(start, _destination, obstacles, _target_speed, _local_planner_path);
}*/

/*void CLocalPlanner::gridmap2localpath(const std::vector<Eigen::Vector2i> &_local_points, const Eigen::MatrixXi& _gridmap, const Eigen::Vector2i& _destination, std::vector<Eigen::VectorXf>& _local_planner_path, std::vector<Eigen::Vector2f>& _fake_obstacles)
{
    _local_planner_path.clear();
    _fake_obstacles.clear();

    CyC_INT nRows = _gridmap.rows();
    CyC_INT nCols = _gridmap.cols();

    m_AstarGenerator.clearCollisions();
    m_AstarGenerator.setWorldSize({ nRows, nCols });
    
    // Add obstacles to the AStar grid
    for (CyC_INT i = 0; i < nRows; ++i)
    {
        for (CyC_INT j = 0; j < nCols; ++j)
        {
            if (_gridmap(i, j) != 0)
            {
                AStar::Vec2i obstacle = { i, j };
                m_AstarGenerator.addCollision(obstacle);
            }
        }
    }

    // Add additional (fake) obstacles to the AStar grid
    for (CyC_INT i = 0; i < nRows; i++)
    {
        for (CyC_INT j = 0; j < nCols; j++)
        {
            for (CyC_INT k = 1; k <= 2; k++)
            {
                if (_gridmap(i, j) != 0)
                {
                    m_AstarGenerator.addCollision({ j + k, i + k });
                    m_AstarGenerator.addCollision({ j + k, i - k });
                    m_AstarGenerator.addCollision({ j - k, i - k });
                    m_AstarGenerator.addCollision({ j - k, i + k });
                    m_AstarGenerator.addCollision({ j + k, i });
                    m_AstarGenerator.addCollision({ j    , i + k });
                    m_AstarGenerator.addCollision({ j - k, i });
                    m_AstarGenerator.addCollision({ j    , i - k });

                    _fake_obstacles.push_back(Eigen::Vector2f(j + k, i + k));
                    _fake_obstacles.push_back(Eigen::Vector2f(j + k, i - k));
                    _fake_obstacles.push_back(Eigen::Vector2f(j - k, i - k));
                    _fake_obstacles.push_back(Eigen::Vector2f(j - k, i + k));
                    _fake_obstacles.push_back(Eigen::Vector2f(j + k, i));
                    _fake_obstacles.push_back(Eigen::Vector2f(j - k, i));
                    _fake_obstacles.push_back(Eigen::Vector2f(j, i + k));
                    _fake_obstacles.push_back(Eigen::Vector2f(j, i - k));
                }
            }
        }
    }
    
    AStar::Vec2i start = { nRows / 2 , nCols / 2 };
    AStar::Vec2i dest = { _destination.x(), _destination.y() };
	std::vector<AStar::Vec2i> local_pts;
	for (auto i : _local_points)
	{
		AStar::Vec2i current_point = { i.x(), i.y() };
		local_pts.push_back(current_point);
	}
    AStar::CoordinateList path = m_AstarGenerator.findPath(local_pts, dest, start);

    for (auto& coordinate : path)
    {
        Eigen::VectorXf pt = Eigen::VectorXf(2);
        pt << (float)coordinate.x, (float)coordinate.y;
        _local_planner_path.push_back(pt);
    }
}*/

/*Eigen::VectorXf CLocalPlanner::getPointInPlannerCoords(const Eigen::Vector4f& _point, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size)
{
    Eigen::VectorXf dest;
    dest.resize(4);
	
	CPose T_globalpath2vehicle(0.F, 0.F, 0.F, 180.F * DEGTORAD, 0.F, 270.F * DEGTORAD - _vehicle_state.y_hat(3));

    float pivot_point_x = _point(0) - _vehicle_state.y_hat(0);
    float pivot_point_y = _point(1) - _vehicle_state.y_hat(1);

    Eigen::Vector4f before_transform(pivot_point_x, pivot_point_y, 0.F, 1.f);
    Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
    after_transform = T_globalpath2vehicle.transform() * before_transform;
    const auto veh_grid_x = (_gridmap_size.x() / _octree_resolution / 2);
    const auto veh_grid_y = (_gridmap_size.y() / _octree_resolution / 2);
    
    dest(0) = veh_grid_x - (CyC_INT)(after_transform(0) / _octree_resolution);
    dest(1) = veh_grid_y - (CyC_INT)(after_transform(1) / _octree_resolution);
    dest(2) = _vehicle_state.y_hat(2);
    dest(3) = _vehicle_state.y_hat(3);
	
    return dest;
}*/

/*Eigen::Vector2i CLocalPlanner::getGoalPointInAStarCoords(const Eigen::Vector4f &_goal_point, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size)
{
    Eigen::Vector2i astar_dest;
	
	CPose T_globalpath2vehicle(0.F, 0.F, 0.F, 180.F * DEGTORAD, 0.F, 270.F * DEGTORAD - _vehicle_state.y_hat(3));

    float pivot_point_x = _goal_point(0) - _vehicle_state.y_hat(0);
    float pivot_point_y = _goal_point(1) - _vehicle_state.y_hat(1);

    Eigen::Vector4f before_transform(pivot_point_x, pivot_point_y, 0.F, 1.f);
    Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
    after_transform = T_globalpath2vehicle.transform() * before_transform;
    const auto veh_astar_grid_x = (_gridmap_size.x() / _octree_resolution / 2);
    const auto veh_astar_grid_y = (_gridmap_size.y() / _octree_resolution / 2);
    
    astar_dest(0) = veh_astar_grid_x - (CyC_INT)(after_transform(0) / _octree_resolution);
    astar_dest(1) =  veh_astar_grid_y - (CyC_INT)(after_transform(1) / _octree_resolution);
    
    return astar_dest;
}
*/

/*std::vector<Eigen::Vector2i> CLocalPlanner::getLocalPointsInAStarCoords(const int &_index, const std::vector<Eigen::Vector4f> &_mission_planner, const int &_offset, const CycStateMeasurement &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size)
{
	std::vector<Eigen::Vector2i> local_points;
	//for (int i = _index; i <= _index + _offset; i+=(_offset / (_offset / 37))) // divide the local path in equal segments each having 37 points
	//{
	//	local_points.push_back(getGoalPointInAStarCoords(_mission_planner.at(i), _vehicle_state, _octree_resolution, _gridmap_size));
	//}
	int current_index = _index;
	for (int i = _index + 1; i < _mission_planner.size(); i++)
	{
		if (sqrtf(powf(_mission_planner.at(i).x() - _mission_planner.at(_index).x(), 2) + powf(_mission_planner.at(i).y() - _mission_planner.at(_index).y(), 2) > 15.0F))
		{
			break;
		}
		//local_points.push_back(getGoalPointInAStarCoords(_mission_planner.at(i), _vehicle_state, _octree_resolution, _gridmap_size));
		//if (sqrtf(powf(_mission_planner.at(i).x() - _mission_planner.at(current_index).x(), 2) + powf(_mission_planner.at(i).y() - _mission_planner.at(current_index).y(), 2) >= 0.01F))
		{
			current_index = i;
			local_points.push_back(getGoalPointInAStarCoords(_mission_planner.at(i), _vehicle_state, _octree_resolution, _gridmap_size));
		}
	}
	if (local_points.empty())
	{
		local_points.push_back(getGoalPointInAStarCoords(_mission_planner.at(_mission_planner.size() - 1), _vehicle_state, _octree_resolution, _gridmap_size));
	}

	return local_points;
}*/

/*std::vector<Eigen::VectorXf> CLocalPlanner::catmullRomChain(std::vector<Eigen::VectorXf> path)
{
	size_t size = path.size();
	if (size < 4)
	{
		return path;
	}
	std::vector<Eigen::VectorXf> control_points;

	// Added an additional point very close to the starting point so the spline will be plotted through the first point
	Eigen::VectorXf start_point = path.at(0);
	start_point[0] -= 0.01f;
	start_point[1] -= 0.01f;
	control_points.push_back(start_point);

	//std::vector<Eigen::VectorXf> control_points = spline_points;
	for (size_t i = 0; i < size; i += (size / 4))
	{
		control_points.push_back(path.at(i));
	}
	control_points.push_back(path.at(path.size() - 1));

	// Added an additional point very close to the end point so the spline will be plotted through the end point
	Eigen::VectorXf end_point = path.at(path.size() - 1);
	end_point[0] += 0.01f;
	end_point[1] += 0.01f;
	control_points.push_back(end_point);

	std::vector<Eigen::VectorXf> new_points;
	for (int i = 0; i < control_points.size() - 3; i++)
	{
		catmullRomSpline(new_points, control_points[i], control_points[i + 1], control_points[i + 2], control_points[i + 3]);
	}
	return new_points;
}*/

/*void CLocalPlanner::catmullRomSpline(std::vector<Eigen::VectorXf> &new_points, Eigen::VectorXf p0, Eigen::VectorXf p1, Eigen::VectorXf p2, Eigen::VectorXf p3)
{
	float alpha = 0.5;
	auto tj = [alpha](float ti, Eigen::VectorXf pi, Eigen::VectorXf pj)
	{
		return powf(sqrtf(powf(pj[0] - pi[0], 2) + powf(pj[1] - pi[1], 2)), alpha) + ti;
	};
	float t0 = 0.0F;
	float t1 = tj(t0, p0, p1);
	float t2 = tj(t1, p1, p2);
	float t3 = tj(t2, p2, p3);
	int number_of_points = 100;
	for (float t = t1; t < t2; t += float((t2 - t1) / float(number_of_points)))
	{
		Eigen::VectorXf a1 = Eigen::VectorXf(2);
		Eigen::VectorXf a2 = Eigen::VectorXf(2);
		Eigen::VectorXf a3 = Eigen::VectorXf(2);
		Eigen::VectorXf b1 = Eigen::VectorXf(2);
		Eigen::VectorXf b2 = Eigen::VectorXf(2);
		Eigen::VectorXf c = Eigen::VectorXf(2);
		a1[0] = (t1 - t) / (t1 - t0) * p0[0] + (t - t0) / (t1 - t0) * p1[0];
		a1[1] = (t1 - t) / (t1 - t0) * p0[1] + (t - t0) / (t1 - t0) * p1[1];
		a2[0] = (t2 - t) / (t2 - t1) * p1[0] + (t - t1) / (t2 - t1) * p2[0];
		a2[1] = (t2 - t) / (t2 - t1) * p1[1] + (t - t1) / (t2 - t1) * p2[1];
		a3[0] = (t3 - t) / (t3 - t2) * p2[0] + (t - t2) / (t3 - t2) * p3[0];
		a3[1] = (t3 - t) / (t3 - t2) * p2[1] + (t - t2) / (t3 - t2) * p3[1];
		b1[0] = (t2 - t) / (t2 - t0) * a1[0] + (t - t0) / (t2 - t0) * a2[0];
		b1[1] = (t2 - t) / (t2 - t0) * a1[1] + (t - t0) / (t2 - t0) * a2[1];
		b2[0] = (t3 - t) / (t3 - t1) * a2[0] + (t - t1) / (t3 - t1) * a3[0];
		b2[1] = (t3 - t) / (t3 - t1) * a2[1] + (t - t1) / (t3 - t1) * a3[1];
		c[0] = (t2 - t) / (t2 - t1) * b1[0] + (t - t1) / (t2 - t1) * b2[0];
		c[1] = (t2 - t) / (t2 - t1) * b1[1] + (t - t1) / (t2 - t1) * b2[1];
		new_points.push_back(c);
	}
}*/

/*std::vector<Eigen::Vector4f> CLocalPlanner::getControlPointsInGlobalCoords(int _index, const std::vector<Eigen::Vector4f> &_mission_planner)
{
	const float goalPointDist = 5.0F;
	std::vector<Eigen::Vector4f> controlPoints;
	for (int i = _index + 1; i < _mission_planner.size(); i++)
	{
		float euclideanDist = sqrtf(powf(_mission_planner.at(i).x() - _mission_planner.at(_index).x(), 2) + powf(_mission_planner.at(i).y() - _mission_planner.at(_index).y(), 2));
		if (euclideanDist > goalPointDist)
		{
			controlPoints.push_back(_mission_planner.at(i));
			break;
		}
		controlPoints.push_back(_mission_planner.at(i));
	}
	if (controlPoints.empty())
	{
		controlPoints.push_back(_mission_planner.at(_mission_planner.size() - 1));
	}
	return controlPoints;
}*/

/*std::vector<float> CLocalPlanner::headingCost(std::vector<Eigen::Vector4f> controlPoints, std::vector<std::vector<Eigen::VectorXf>> candidates)
{
	std::vector<float> costVector(candidates.size(), 0);
	for (int i = 0; i < controlPoints.size(); i++)
	{
		int trajIdx = 0;
		for (auto trajectory : candidates)
		{
			int trajPoints = trajectory.size() - 1;
			Eigen::VectorXf pointToEvaluate = trajectory.at((trajPoints * (i)) / controlPoints.size());
			float euclideanDist = sqrtf(powf(controlPoints.at(i).x() - pointToEvaluate.x(), 2) + powf(controlPoints.at(i).y() - pointToEvaluate.y(), 2));
			costVector[trajIdx] += euclideanDist;
			trajIdx++;
		}
	}
	return costVector;
}*/

/*std::vector<float> CLocalPlanner::obstacleCost(std::vector<Eigen::Vector4f> obstaclePoints, std::vector<std::vector<Eigen::VectorXf>> candidates)
{
	float obstacleBiasCoefficient = 10000.F;
	std::vector<float> obstacleCosts(candidates.size(), 0);
	int trajIdx = 0;
	if (!obstaclePoints.empty())
	{
		for (auto trajectory : candidates)
		{
			Eigen::VectorXf originState = trajectory.at(0);
			for (auto pt : trajectory)
			{
				for (auto obst : obstaclePoints)
				{
					if (detectCollision(pt, obst))
					{
						auto euclideanDist = [](Eigen::VectorXf pt1, Eigen::VectorXf pt2)
						{
							return sqrtf(powf(pt1.x() - pt2.x(), 2) + powf(pt1.y() - pt2.y(), 2));
						};
						obstacleCosts[trajIdx] += obstacleBiasCoefficient - euclideanDist(pt, originState);
						//obstacleCosts[trajIdx] = FLT_MAX;
						break;
					}
				}
			}
			trajIdx++;
		}
	}
	return obstacleCosts;
}*/

/*std::vector<int> CLocalPlanner::selectParetoOptimalSolutions(std::vector<float> obstacleCosts, std::vector<float> headingCosts)
{
	std::vector<int> paretoIdxs;
	for (int i = 0; i < obstacleCosts.size(); i++)
	{
		bool isOptimal = true;
		for (int j = 0; j < obstacleCosts.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			if ((obstacleCosts[i] >= obstacleCosts[j]) && (headingCosts[i] >= headingCosts[j]))
			{
				isOptimal = false;
				break;
			}
		}
		if (isOptimal)
		{
			paretoIdxs.push_back(i);
		}
	}
	return paretoIdxs;
}*/

/*void CLocalPlanner::discardInvalidTrajectories(std::vector<std::vector<Eigen::VectorXf>>& candidates, Eigen::Vector4f goalPt, float laneWidth)
{
	auto euclideanDist = [](Eigen::Vector4f pt1, Eigen::VectorXf pt2)
	{
		return sqrtf(powf(pt1.x() - pt2.x(), 2) + powf(pt1.y() - pt2.y(), 2));
	};
	int sampleIdx = 0;
	for (auto sample : candidates)
	{
		if (!sample.empty())
		{
			Eigen::VectorXf finalTrajPt = sample.at(sample.size() - 1);
			float goalDist = euclideanDist(goalPt, finalTrajPt);
			if (goalDist > laneWidth / 2.0F + 0.1F)
			{
				candidates.erase(candidates.begin() + sampleIdx);
				sampleIdx--;
			}
			sampleIdx++;
		}
	}
}*/

/*bool CLocalPlanner::detectCollision(Eigen::VectorXf state, Eigen::Vector4f obstaclePt)
{
	float vehicleLength = 4.3F;
	float vehicleWidth = 2.3F;
	float vehicleX1 = state.x() - vehicleLength / 2.0F;
	float vehicleY1 = state.y() - vehicleWidth / 2.0F;
	float vehicleX2 = state.x() + vehicleLength / 2.0F;
	float vehicleY2 = state.y() + vehicleWidth / 2.0F;
	return (obstaclePt.x() > vehicleX1) && (obstaclePt.y() > vehicleY1) && (obstaclePt.x() < vehicleX2) && (obstaclePt.y() < vehicleY2);
}*/
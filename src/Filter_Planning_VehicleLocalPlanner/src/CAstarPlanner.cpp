// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CAstarPlanner.h"
#include "env/COcTreeUtils.h"
#include "plan/CPlanningUtils.h"

CAstarPlanner::CAstarPlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt):
    m_VehicleModelFile(_vehicle_model_file),
    m_LookaheadDistance(_lookahead_distance),
    m_Dt(_dt)
{
    m_AstarGenerator.setHeuristic(AStar::Heuristic::euclidean);
    m_AstarGenerator.setDiagonalMovement(true);
}

CAstarPlanner::~CAstarPlanner()
{}

std::vector<Eigen::Vector2i> CAstarPlanner::getLocalPointsInAStarCoords(const size_t &_index, const std::vector<Eigen::Vector4f> &_mission_planner, const int &_offset, const CycState &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size)
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
}

CycReferenceSetPoints CAstarPlanner::perception2referenceAStar(const std::vector<Eigen::Vector4f> &_mission_path, const CcrOcTree& _octree, const CycState &_vehicle_state)
{
    CycReferenceSetPoints reference_setpoints;

    // Calculate the local reference path using the environment model and A*
    Eigen::MatrixXi gridmap;
    //Eigen::Vector2f gridmap_size{ 12.f, 12.f };
    COcTreeUtils::octree2gridmap(_octree, gridmap, m_gridMapSize);

    std::vector<Eigen::VectorXf> astar_local_path;
    std::vector<Eigen::Vector2f> _fakeObstacles;

    // Find the goal point
    auto index = CPlanningUtils::findClosestPoint(_mission_path, _vehicle_state, m_PreviousTrajectoryPointIndex);
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
    CPose T_astar2world(_vehicle_state.x_hat(0), _vehicle_state.x_hat(1), 0.f, 0.F, 0.F, 90.F * DEG2RAD + _vehicle_state.x_hat(3));

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
}

std::vector<Eigen::VectorXf> CAstarPlanner::catmullRomChain(std::vector<Eigen::VectorXf> path)
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
}

void CAstarPlanner::catmullRomSpline(std::vector<Eigen::VectorXf> &new_points, Eigen::VectorXf p0, Eigen::VectorXf p1, Eigen::VectorXf p2, Eigen::VectorXf p3)
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
}

Eigen::Vector2i CAstarPlanner::getGoalPointInAStarCoords(const Eigen::Vector4f &_goal_point, const CycState &_vehicle_state, const float& _octree_resolution, const Eigen::Vector2f& _gridmap_size)
{
    Eigen::Vector2i astar_dest;

    CPose T_globalpath2vehicle(0.F, 0.F, 0.F, 180.F * DEG2RAD, 0.F, 270.F * DEG2RAD - _vehicle_state.x_hat(3));

    float pivot_point_x = _goal_point(0) - _vehicle_state.x_hat(0);
    float pivot_point_y = _goal_point(1) - _vehicle_state.x_hat(1);

    Eigen::Vector4f before_transform(pivot_point_x, pivot_point_y, 0.F, 1.f);
    Eigen::Vector4f after_transform(0.F, 0.F, 0.F, 1.f);
    after_transform = T_globalpath2vehicle.transform() * before_transform;
    const auto veh_astar_grid_x = (_gridmap_size.x() / _octree_resolution / 2);
    const auto veh_astar_grid_y = (_gridmap_size.y() / _octree_resolution / 2);

    astar_dest(0) = veh_astar_grid_x - (CyC_INT)(after_transform(0) / _octree_resolution);
    astar_dest(1) = veh_astar_grid_y - (CyC_INT)(after_transform(1) / _octree_resolution);

    return astar_dest;
}

void CAstarPlanner::gridmap2localpath(const std::vector<Eigen::Vector2i> &_local_points, const Eigen::MatrixXi& _gridmap, const Eigen::Vector2i& _destination, std::vector<Eigen::VectorXf>& _local_planner_path, std::vector<Eigen::Vector2f>& _fake_obstacles)
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
}

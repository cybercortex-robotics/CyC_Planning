// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CLatticePlanner.h"
#include "env/COcTreeUtils.h"
#include "plan/CPlanningUtils.h"


CLatticePlanner::CLatticePlanner(const std::string& _vehicle_model_file, const float& _lookahead_distance, const float& _dt) :
    m_LookaheadDistance(_lookahead_distance)
{
    m_pTrajectoryGenerator = std::make_unique<CTrajectoryGenerator>(_vehicle_model_file, _dt);
    m_pVehicleModel = std::make_unique<VehicleModel>(_vehicle_model_file);
}

CLatticePlanner::~CLatticePlanner()
{}

CycReferenceSetPoints CLatticePlanner::perception2referenceLP(const std::vector<Eigen::Vector4f>& _mission_path, const CcrOcTree & _octree, const CycState & _vehicle_state)
{
    CycReferenceSetPoints reference_setpoints;

    // Calculate the local reference path using the environment model
    Eigen::MatrixXi gridmap;
    //Eigen::Vector2f gridmap_size{ 3.f, 3.f };
    COcTreeUtils::octree2gridmap(_octree, gridmap, m_gridMapSize);

    // Find the goal point
    auto index = CPlanningUtils::findClosestPoint(_mission_path, _vehicle_state, m_PreviousTrajectoryPointIndex);
    m_PreviousTrajectoryPointIndex = index < _mission_path.size() - 1 ? index : 0;

    auto vel = _mission_path[index][2];

    Eigen::Vector2f startPoint;
    Eigen::Vector2f goalPoint;
    startPoint << _vehicle_state.x_hat(0), _vehicle_state.x_hat(1);
    std::vector<Eigen::Vector4f> controlPoints;
    controlPoints = getControlPointsInGlobalCoords(index, _mission_path);
    goalPoint << controlPoints.back()[0], controlPoints.back()[1];

    // Rotation around the Z axis with the yaw angle of the car
    CPose T_globalpath2vehicle(0.0f, 0.0f, 0.0F, 0.0F, 0.0F, -_vehicle_state.x_hat(3));
    Eigen::Vector4f beforeTransformStart(_vehicle_state.x_hat(0), _vehicle_state.x_hat(1), 0.0F, 1.f);
    Eigen::Vector4f afterTransformStart(0.0F, 0.0F, 0.0F, 1.f);
    Eigen::Vector4f beforeTransformGoal(goalPoint(0), goalPoint(1), 0.0F, 1.f);
    Eigen::Vector4f afterTransformGoal(0.0F, 0.0F, 0.0F, 1.f);
    afterTransformStart = T_globalpath2vehicle.transform() * beforeTransformStart;
    afterTransformGoal = T_globalpath2vehicle.transform() * beforeTransformGoal;


    // Generate candidate trajectories
    //float angle = controlPoints.back()[3] -_vehicle_state.y_hat[3];
    //float angle = 0.f;

    float angle = acosf((afterTransformStart.x() * afterTransformGoal.x() + afterTransformStart.y() * afterTransformGoal.y()) / 
                        (sqrtf(afterTransformStart.x() * afterTransformStart.x() + afterTransformStart.y() * afterTransformStart.y()) * 
                            sqrtf(afterTransformGoal.x() * afterTransformGoal.x() + afterTransformGoal.y() * afterTransformGoal.y())));

    float laneWidth = 3.0F;
    m_pTrajectoryGenerator->setState(afterTransformStart(0), afterTransformStart(1), afterTransformGoal(0), afterTransformGoal(1), angle, _vehicle_state.x_hat[2]);

    //CTrajectoryGenerator trajectoryGenerator(afterTransformStart(0), afterTransformStart(1), afterTransformGoal(0), afterTransformGoal(1), angle);
    
    std::vector<std::vector<Eigen::VectorXf>> laneSamples = m_pTrajectoryGenerator->laneStateSampling(laneWidth);

    // Rotate again around the Z axis to revert to the world coordinates system
    CPose T_vehicle2globalpath(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, _vehicle_state.x_hat(3));
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
    T_vehicle2globalpath.update(_vehicle_state.x_hat(0), _vehicle_state.x_hat(1), 0.0F, 0.0F, 0.0F, _vehicle_state.x_hat(3));
    std::vector<Eigen::Vector4f> obstacles;
    for (CcrOcTree::leaf_iterator it = _octree.begin_leafs(); it != _octree.end_leafs(); it++)
    {
        Eigen::Vector4f obstaclePt;
        obstaclePt << it.getX(), it.getY(), it.getZ(), 1.f;
        Eigen::Vector4f worldObstaclePt = T_vehicle2globalpath.transform() * obstaclePt;
        obstacles.push_back(worldObstaclePt);
    }

    discardInvalidTrajectories(laneSamples, afterTransformGoal, laneWidth);

    //bestTrajIdx = headingCost(controlPoints, laneSamples);
    std::vector<float> headingCosts = headingCost(controlPoints, laneSamples);
    std::vector<float> obstacleCosts = obstacleCost(obstacles, laneSamples);
    
    //std::vector<CyC_INT> paretoOptimalSolutions = selectParetoOptimalSolutions(obstacleCosts, headingCosts);

    //spdlog::info("========================================");
    float minCost = FLT_MAX;
    int bestIdx = 0;
    for (int i = 0; i < headingCosts.size(); i++)
    {
        float cost = headingCosts[i] + obstacleCosts[i];
        if (cost < minCost)
        {
            bestIdx = i;
            minCost = cost;
        }
    }

    if (laneSamples.size() > bestIdx)
    {
        for (auto sample : laneSamples[bestIdx])
        {
            Eigen::Vector4f pt;
            pt << sample[0], sample[1], vel, 0.f;
            reference_setpoints.ref.push_back(pt);
        }
        //reference_setpoints.ref = laneSamples.at(bestIdx);
        reference_setpoints.ref_samples = laneSamples;
    }
    else
    {
        spdlog::warn("Could not generate lattice");
    }
    return reference_setpoints;
}

std::vector<Eigen::Vector4f> CLatticePlanner::getControlPointsInGlobalCoords(size_t _index, const std::vector<Eigen::Vector4f> &_mission_planner)
{
    const float goalPointDist = m_LookaheadDistance;
    std::vector<Eigen::Vector4f> controlPoints;
    for (size_t i = _index + 1; i < _mission_planner.size(); i++)
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
}

std::vector<float> CLatticePlanner::headingCost(std::vector<Eigen::Vector4f> controlPoints, std::vector<std::vector<Eigen::VectorXf>> candidates)
{
    std::vector<float> costVector(candidates.size(), 0);
    for (int i = 0; i < controlPoints.size(); i++)
    {
        int trajIdx = 0;
        for (auto trajectory : candidates)
        {
            size_t trajLength = trajectory.size() - 1;
            Eigen::VectorXf pointToEvaluate = trajectory[(trajLength * i) / controlPoints.size()];
            float euclideanDist = sqrtf(powf(controlPoints[i].x() - pointToEvaluate.x(), 2) + powf(controlPoints[i].y() - pointToEvaluate.y(), 2));
            costVector[trajIdx] += euclideanDist;
            trajIdx++;
        }
    }
    return costVector;
}

std::vector<float> CLatticePlanner::obstacleCost(std::vector<Eigen::Vector4f> obstaclePoints, std::vector<std::vector<Eigen::VectorXf>> candidates)
{
    float obstacleBiasCoefficient = 1e6F;
    std::vector<float> obstacleCosts(candidates.size(), 0);
    int trajIdx = 0;
    if (!obstaclePoints.empty())
    {
        for (auto trajectory : candidates)
        {
            bool foundObstacle = false;
            Eigen::VectorXf originState = trajectory.at(0);
            for (auto pt : trajectory)
            {
                for (auto obst : obstaclePoints)
                {
                    if (detectCollision(pt, obst))
                    {
                        foundObstacle = true;
                        obstacleCosts[trajIdx] = obstacleBiasCoefficient;
                        break;
                    }
                }
                if (foundObstacle)
                    break;
            }
            trajIdx++;
        }
    }
    return obstacleCosts;
}

void CLatticePlanner::discardInvalidTrajectories(std::vector<std::vector<Eigen::VectorXf>>& candidates, Eigen::Vector4f goalPt, float laneWidth)
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
}

bool CLatticePlanner::detectCollision(Eigen::VectorXf state, Eigen::Vector4f obstaclePt)
{
    float vehicleLength = m_pVehicleModel->m_fLongDistWheels;
    float vehicleWidth = m_pVehicleModel->m_fLatDistWheels;
    float vehicleX1 = state.x() - vehicleLength / 2.0F;
    float vehicleY1 = state.y() - vehicleWidth / 2.0F;
    float vehicleX2 = state.x() + vehicleLength / 2.0F;
    float vehicleY2 = state.y() + vehicleWidth / 2.0F;
    return (obstaclePt.x() > vehicleX1) && (obstaclePt.y() > vehicleY1) && (obstaclePt.x() < vehicleX2) && (obstaclePt.y() < vehicleY2);
}
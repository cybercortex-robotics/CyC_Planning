// Copyright (c) 2024 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CTRAJECTORYGENERATOR_H_
#define CTRAJECTORYGENERATOR_H_

#include "CyC_TYPES.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "spline.h"
#include "control/CStateSpaceModelVehicle.h"

struct TrajectoryState
{
	float x;
	float y;
	float yaw;
	float v;

	TrajectoryState(float x, float y, float yaw, float v)
	{
		this->x = x;
		this->y = y;
		this->yaw = yaw;
		this->v = v;
	}

	void update(float v, float delta, float dt, float L)
	{
		this->v = v;
		this->x = this->x + this->v * cosf(this->yaw) * dt;
		this->y = this->y + this->v * sinf(this->yaw) * dt;
		this->yaw = this->yaw - this->v / L * tan(delta) * dt;
	}
};

class CTrajectoryGenerator
{
public:
    CTrajectoryGenerator(const std::string& _vehicle_model_file, float dt);

	// Generates paths based on a state lattice planner
	std::vector<std::vector<Eigen::VectorXf>> laneStateSampling(float _laneWidth);

    void setState(float start_x, float start_y, float target_x, float target_y, float heading, float velocity);

private:
	float L; // wheel base
	float ds; // course distance
	float v; // velocity
	float start_x;
	float start_y;
	float target_x;
	float target_y;
	float heading;
    std::unique_ptr<VehicleModel>   m_pVehicleModel;

	// Calculates list of target states
	std::vector<Eigen::Vector3f> calculateStatesList(float laneCenter, float laneHeading, float laneWidth, float vehicleWidth, float d, float nxy);

	// Optimizes the generated candidate trajectory with the purpose of it being as close as possible to the target state
	std::vector<std::vector<float>> optimizeTrajectory(TrajectoryState target, float k0, Eigen::Vector3f &p, bool &isValid);

	// Generates a candidate trajectory
	std::vector<std::vector<float>> generateTrajectory(float s, float km, float kf, float k0);

	// C++ equivalent to numpy.arange
	void arange(std::vector<float> &vect, float start, float end, float step);

	// Calculates the difference between the target state and a candidate trajectory, therefore helps cost computing
	Eigen::Vector3f calculateDifference(TrajectoryState target, std::vector<std::vector<float>> trajectory);

	// Cost minimization function
	Eigen::Matrix3f calcJ(TrajectoryState target, Eigen::VectorXf p, Eigen::Vector3f h, float k0);

	// Returns candidate trajectories
	std::vector<std::vector<Eigen::VectorXf>> generatePaths(std::vector<Eigen::Vector3f> targetStates, float k0);

	// Plots the generated candidate trajectories for test purposes
	//void plotTrajectories(std::vector<Eigen::VectorXf> lookupTable, float k0);
};
#endif /*CTRAJECTORYGENERATOR_H_ */

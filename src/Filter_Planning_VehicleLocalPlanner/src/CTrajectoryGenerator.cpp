// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CTrajectoryGenerator.hpp"

CTrajectoryGenerator::CTrajectoryGenerator(const std::string& _vehicle_model_file, float dt)
{
    this->m_pVehicleModel = std::make_unique<VehicleModel>(_vehicle_model_file);
    this->L = m_pVehicleModel->m_fLongDistWheels;
    this->ds = dt;
    this->v = 10.f / 3.6f;
}

void CTrajectoryGenerator::setState(float start_x, float start_y, float target_x, float target_y, float heading, float velocity)
{
    if (velocity != 0.f)
        this->v = velocity;
    this->start_x = start_x;
    this->start_y = start_y;
    this->target_x = target_x;
    this->target_y = target_y;
    this->heading = heading;
}

std::vector<std::vector<Eigen::VectorXf>> CTrajectoryGenerator::laneStateSampling(float _laneWidth)
{
	float k0 = 0.0F;
	float laneCenter = target_y; // lateral position
    float laneHeading = heading;// *DEGTORAD;
	float laneWidth = _laneWidth;
    float vehicleWidth = m_pVehicleModel->m_fLatDistWheels;//0.0F;
	float d = target_x; // longitudinal position
	float nxy = 5; // number of candidate trajectories
	std::vector<Eigen::Vector3f> states = calculateStatesList(laneCenter, laneHeading, laneWidth, vehicleWidth, d, nxy);
	return generatePaths(states, k0);
}

std::vector<Eigen::Vector3f> CTrajectoryGenerator::calculateStatesList(float laneCenter, float laneHeading, float laneWidth, float vehicleWidth, float d, float nxy)
{
	//float xc = cosf(laneHeading) * d + sinf(laneHeading) * laneCenter;
	//float yc = sinf(laneHeading) * d + cosf(laneHeading) * laneCenter;
	float xc = d;
	float yc = laneCenter;
	std::vector<Eigen::Vector3f> states;

	for (int i = 0; i < nxy; i++)
	{
		float delta = -0.5 * (laneWidth - vehicleWidth) + (laneWidth - vehicleWidth) * i / (nxy - 1);
		float xf = xc - delta * sinf(laneHeading);
		float yf = yc + delta * cosf(laneHeading);
		float yawf = laneHeading;
		Eigen::Vector3f state;
		state << xf, yf, yawf;
		states.push_back(state);
	}

	/*spdlog::info("========================");
	for (auto state : states)
	{
		spdlog::info("Final state: {} {}", state(0), state(1));
	}
	spdlog::info("========================");*/

	return states;
}

std::vector<std::vector<float>> CTrajectoryGenerator::optimizeTrajectory(TrajectoryState target, float k0, Eigen::Vector3f &p, bool &isValid)
{
	std::vector<std::vector<float>> generatedTrajectory;
	const float cost_threshold = 0.1f; // arbitrarily chosen, dictates the indulgence regarding the trajectory reaching it's target state
	for (int i = 0; i < 100; i++)
	{
		generatedTrajectory = generateTrajectory(p[0], p[1], p[2], k0);
		Eigen::Vector3f dc = calculateDifference(target, generatedTrajectory);
		float cost = dc.norm(); // the cost is the norm of the differences between x, y and yaw
		if (cost < cost_threshold)
		{
			break;
		}
		Eigen::Vector3f h;
		Eigen::Matrix3f J = calcJ(target, p, h, k0);
		if (J.determinant() == 0)
		{
			isValid = false;
            //spdlog::info("Invalid trajectory!");
            break;
		}
		Eigen::MatrixXf dcTranspose(3, 1);
		dcTranspose.col(0) = dc;
		Eigen::Vector3f dp = -J.inverse() * dcTranspose;
		float alpha = 1.0; // learning rate
		p = p + (alpha * dp); // adjust parameters
	}
	return generatedTrajectory;
}

std::vector<std::vector<float>> CTrajectoryGenerator::generateTrajectory(float s, float km, float kf, float k0)
{
	float n = s / ds;
	float time = s / v;
	std::vector<float> tk = { 0.0F, time / 2.0F, time };
	std::vector<float> kk = { k0, km, kf };
	std::vector<float> t;
	arange(t, 0.0F, time, time/n);

	// Spline generation dependent on time
	tk::spline fkp;
	fkp.set_points(tk, kk);
	std::vector<float> kp;
	for (float ti : t)
	{
		kp.push_back(fkp(ti));
	}
	float dt = time / n;

	// Computing the generated trajectory
	TrajectoryState state(this->start_x, this->start_y, 0.0F, 0.0F);
	std::vector<float> x = { state.x };
	std::vector<float> y = { state.y };
	std::vector<float> yaw = { state.yaw };
	for (float ikp : kp)
	{
		state.update(v, ikp, dt, this->L);
		x.push_back(state.x);
		y.push_back(state.y);
		yaw.push_back(state.yaw);
	}
	std::vector<std::vector<float>> resultTrajectory = { x, y, yaw };
	return resultTrajectory;
}

void CTrajectoryGenerator::arange(std::vector<float> & vect, float start, float end, float step)
{
	for (float value = start; value < end; value += step)
	{
		vect.push_back(value);
	}
}

Eigen::Vector3f CTrajectoryGenerator::calculateDifference(TrajectoryState target, std::vector<std::vector<float>> trajectory)
{
	float xd = target.x - trajectory[0][trajectory[0].size() - 1];
	float yd = target.y - trajectory[1][trajectory[1].size() - 1];
	float yawd = target.yaw - trajectory[2][trajectory[2].size() - 1];
	Eigen::Vector3f difference;
	difference << xd, yd, yawd;
	return difference;
}

Eigen::Matrix3f CTrajectoryGenerator::calcJ(TrajectoryState target, Eigen::VectorXf p, Eigen::Vector3f h, float k0)
{
	h << 0.5F, 0.02F, 0.02F;
	std::vector<std::vector<float>> positive = generateTrajectory(p[0] + h[0], p[1], p[2], k0);
	Eigen::Vector3f dp = calculateDifference(target, positive);
	std::vector<std::vector<float>> negative = generateTrajectory(p[0] - h[0], p[1], p[2], k0);
	Eigen::Vector3f dn = calculateDifference(target, negative);
	Eigen::Vector3f d1 = (dp - dn) / (2 * h[0]);

	positive = generateTrajectory(p[0], p[1] + h[1], p[2], k0);
	dp = calculateDifference(target, positive);
	negative = generateTrajectory(p[0], p[1] - h[1], p[2], k0);
	dn = calculateDifference(target, negative);
	Eigen::Vector3f d2 = (dp - dn) / (2 * h[1]);

	positive = generateTrajectory(p[0], p[1], p[2] + h[2], k0);
	dp = calculateDifference(target, positive);
	negative = generateTrajectory(p[0], p[1], p[2] - h[2], k0);
	dn = calculateDifference(target, negative);
	Eigen::Vector3f d3 = (dp - dn) / (2 * h[2]);

	Eigen::Matrix3f J;
	J.col(0) = d1;
	J.col(1) = d2;
	J.col(2) = d3;

	return J;
}

std::vector<std::vector<Eigen::VectorXf>> CTrajectoryGenerator::generatePaths(std::vector<Eigen::Vector3f> targetStates, float k0)
{
	Eigen::VectorXf lookupTable = Eigen::VectorXf(6);
	lookupTable << 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F;
	std::vector<Eigen::VectorXf> trajectoryPool;

	for (auto state : targetStates)
	{
		TrajectoryState targetState(state[0], state[1], state[2], 0.0F);
		Eigen::Vector3f p;
		p << sqrtf(powf(state[0], 2) + powf(state[1], 2)), lookupTable[4], lookupTable[5];
		bool isValid = true;
		std::vector <std::vector<float>> optimizedTrajectory = optimizeTrajectory(targetState, k0, p, isValid);
		if (!isValid)
		{
			continue;
		}
		Eigen::VectorXf proposedTrajectory = Eigen::VectorXf(6);
		int lastPtIdx = (int)(optimizedTrajectory[0].size() - 1);
		float x = optimizedTrajectory[0][lastPtIdx];
		float y = optimizedTrajectory[1][lastPtIdx];
		float yaw = optimizedTrajectory[2][lastPtIdx];
		proposedTrajectory << x, y, yaw, p[0], p[1], p[2];
		trajectoryPool.push_back(proposedTrajectory);
	}
	
	std::vector<std::vector<Eigen::VectorXf>> candidateTrajectories;
	//candidateTrajectories.push_back(trajectoryPool);
	for (auto trajectory : trajectoryPool)
	{
		std::vector<Eigen::VectorXf> trajectoryPoints;
		std::vector<std::vector<float>> rawTrajectory = generateTrajectory(trajectory[3], trajectory[4], trajectory[5], k0);
		for (int i = 0; i < rawTrajectory[0].size(); i++)
		{
			Eigen::VectorXf trajectoryPoint = Eigen::VectorXf(2);
			trajectoryPoint << rawTrajectory[0][i], rawTrajectory[1][i];
			trajectoryPoints.push_back(trajectoryPoint);
		}
		candidateTrajectories.push_back(trajectoryPoints);
	}

	return candidateTrajectories;
}

//void CTrajectoryGenerator::plotTrajectories(std::vector<Eigen::VectorXf> lookupTable, float k0)
//{
//	std::string name = "traj";
//	cvplot::setWindowTitle(name, "Trajectories test");
//	cvplot::resizeWindow(name, 1024, 720);
//	auto &figure = cvplot::figure(name);
//	figure.origin(true, true);
//
//	// Calculating the trajectories from the lookup table and plotting them
//	for (Eigen::VectorXf proposal : lookupTable)
//	{
//		std::vector<std::pair<float, float>> splinePoints;
//		std::vector<std::vector<float>>rawTrajectory = generateTrajectory(proposal(3), proposal(4), proposal(5), k0);
//		for (int i = 0; i < rawTrajectory[0].size(); i++)
//		{
//			splinePoints.push_back(std::pair<float, float>(rawTrajectory[0][i], rawTrajectory[1][i]));
//		}
//		figure.series("spline").add(splinePoints).type(cvplot::Dots).color(cvplot::Red);
//	}
//	figure.show(true);
//	cv::waitKey();
//}



#include "CPlannerCMNST.h"
#include <math/CPolynomialFitting.h>
#include <random>
#include <spdlog/spdlog.h>

#ifdef HAS_SPLINE
#include <unsupported/Eigen/Splines>

using Spline3f = Eigen::Spline<float, 3>;

CycTrajectory generate_spline_trajectory(
    const CMNSTConfig& config,
    const Eigen::VectorXf& xvals,
    const Eigen::VectorXf& yvals,
    const Eigen::VectorXf& zvals)
{
    const float sample_step = config.longitudinal_distance / config.num_samples;

    Eigen::MatrixXf points = Eigen::MatrixXf::Zero(3, xvals.size());
    points.row(0) = xvals;
    points.row(1) = yvals;
    points.row(2) = zvals;

    Spline3f spline = Eigen::SplineFitting<Spline3f>::Interpolate(points, config.num_waypoints - 1);

    CycTrajectory trajectory;
    for (CyC_INT i = 0; i < config.num_samples; ++i)
    {
        const float t = i * sample_step / config.longitudinal_distance;
        trajectory.push_back(spline(t));
    }

    return trajectory;
}

#endif // HAS_SPLINE

CycTrajectory generate_polynomial_trajectory(
    const CMNSTConfig& config,
    const Eigen::VectorXf& xvals,
    const Eigen::VectorXf& yvals,
    const Eigen::VectorXf& zvals)
{
    const float sample_step = config.longitudinal_distance / config.num_samples;

    const Eigen::VectorXf poly_y = CPolynomialFitting::polyfit(xvals, yvals, config.num_waypoints - 1);
    const Eigen::VectorXf poly_z = CPolynomialFitting::polyfit(xvals, zvals, config.num_waypoints - 1);

    CycTrajectory trajectory;
    for (CyC_INT i = 0; i < config.num_samples; ++i)
    {
        const float x = i * sample_step;
        const float y = CPolynomialFitting::polyeval(poly_y, x);
        const float z = CPolynomialFitting::polyeval(poly_z, x);

        Eigen::VectorXf point = Eigen::VectorXf::Zero(3);
        point << x, y, z;
        trajectory.emplace_back(std::move(point));
    }

    return trajectory;
}


namespace {
    class PRNG
    {
    public:
        PRNG()
            : gen{ rd() }
        {
        }

        PRNG(const PRNG&) = delete;
        PRNG(PRNG&&) = delete;
        PRNG& operator=(const PRNG&) = default;
        PRNG& operator=(PRNG&&) = default;
        ~PRNG() = default;

        float next(float var)
        {
            std::normal_distribution distr{0.F, var};
            return distr(gen);
        }

    private:
        std::random_device rd;
        std::mt19937 gen;
    };

    float CyC_rand(float var)
    {
        static PRNG prng;
        return prng.next(var);
    }
}

void getCustomParameter(CyC_INT& value, const CPlannerCMNST::ParametersType& parameters, const std::string& name)
{
    auto it = parameters.find(name);
    if (it == parameters.end())
    {
        return;
    }

    const auto& param = it->second;

    char* ptr_end = nullptr;
    const CyC_INT tmpVal = std::strtol(param.c_str(), &ptr_end, 10);
    if (ptr_end != param.c_str())
    {
        value = tmpVal;
    }
    else
    {
        spdlog::warn("CPlannerCMNST: Failed to convert parameter to int: {}", param);
    }
}

void getCustomParameter(float& value, const CPlannerCMNST::ParametersType& parameters, const std::string& name)
{
    auto it = parameters.find(name);
    if (it == parameters.end())
    {
        return;
    }

    const auto& param = it->second;

    char* ptr_end = nullptr;
    const float tmpVal = std::strtof(param.c_str(), &ptr_end);
    if (ptr_end != param.c_str())
    {
        value = tmpVal;
    }
    else
    {
        spdlog::warn("CPlannerCMNST: Failed to convert parameter to float: {}", param);
    }
}

void getCustomParameter(bool& value, const CPlannerCMNST::ParametersType& parameters, const std::string& name)
{
    auto it = parameters.find(name);
    if (it == parameters.end())
    {
        return;
    }

    const auto& param = it->second;
    value = (param == "true");
}

void CPlannerCMNST::loadConfiguration(const ParametersType& parameters)
{
    getCustomParameter(m_config.num_layers, parameters, "config.layers");
    getCustomParameter(m_config.num_samples, parameters, "config.samples");
    getCustomParameter(m_config.num_vertices, parameters, "config.vertices");
    getCustomParameter(m_config.layer_distance, parameters, "config.layer_distance");
    getCustomParameter(m_config.layer_factor, parameters, "config.layer_factor");
    getCustomParameter(m_config.longitudinal_distance, parameters, "config.longitudinal_distance");
    getCustomParameter(m_config.offset, parameters, "config.offset");
    getCustomParameter(m_config.vertex_factor, parameters, "config.vertex_factor");
    getCustomParameter(m_config.use_spline, parameters, "config.use_spline");

#ifndef HAS_SPLINE
    if (m_config.use_spline)
    {
        spdlog::warn("CPlannerCMNST: Spline was enabled, but the Eigen header is not available! Defaulting to polynomial trajectory.");
    }
#endif
}

static size_t find_closest_point(
    const std::vector<Eigen::Vector4f>& trajectory,
    const Eigen::VectorXf& state)
{
    const Eigen::Vector3f position = state.head<3>();
    size_t closest_idx = 0;
    float closest_dist = (trajectory.front().head<3>() - position).norm();
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        const float dist = (trajectory[i].head<3>() - position).norm();
        if (dist < closest_dist)
        {
            closest_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}

CycTrajectory CPlannerCMNST::get_reference_waypoints(
    const std::vector<Eigen::Vector4f>& trajectory,
    const Eigen::VectorXf& state)
{
    const float longitudinal_step = m_config.longitudinal_distance / m_config.num_waypoints;

    std::vector<Eigen::Vector4f> waypoints;

    // find closest point on trajectory to the drone
    const size_t closest_point_found = find_closest_point(trajectory, state);

    // if the last point is somehow ahead of the one found now, then use that one to avoid getting
    // back on the trajectory
    const size_t closest_point = std::max(closest_point_found, m_last_closest_point_idx);
    m_last_closest_point_idx = closest_point;

    // add first waypoint as the closest point
    waypoints.push_back(trajectory[closest_point]);

    // find the following waypoints
    std::vector<float> distances; // keep distances between waypoints, used on spline
    size_t point_idx = closest_point;
    while (waypoints.size() < m_config.num_waypoints)
    {
        const float threshold = longitudinal_step * waypoints.size();

        float dist = 0.F;
        while ((dist < threshold) && (point_idx < trajectory.size()))
        {
            const size_t last_point_idx = (point_idx > 0) ? (point_idx - 1) : 0;
            dist += (trajectory[point_idx].head<3>() - trajectory[last_point_idx].head<3>()).norm();
            point_idx++;
        }

        waypoints.push_back(trajectory[point_idx]);
        distances.push_back(dist);
    }

    const float total_distance = std::accumulate(distances.begin(), distances.end(), 0.F);

    // make sure the 4th value in vector is 1 because we'll transform the points
    for (auto& point : waypoints)
    {
        point.w() = 1.F;
    }

    // drone state:
    // x: = [x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc, y_acc, z_acc, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel] ^ T

    // transform the points relative to the first waypoint as position, and
    // relative to the drone rotation
    const Eigen::Vector3f translation = waypoints.front().head<3>();
    const Eigen::Vector3f rotation = state.segment<3>(9);
    const CPose pose{ translation, rotation };
    const Eigen::Matrix4f T = pose.transform().inverse();

    for (auto& point : waypoints)
    {
        point = T * point;
    }

    Eigen::MatrixXf points = Eigen::MatrixXf::Zero(3, waypoints.size());
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        points.row(0)[i] = waypoints[i].x();
        points.row(1)[i] = waypoints[i].y();
        points.row(2)[i] = waypoints[i].z();
    }

    Spline3f spline = Eigen::SplineFitting<Spline3f>::Interpolate(points, m_config.num_waypoints - 1);

    CycTrajectory out_waypoints;
    out_waypoints.push_back(spline(0.F));
    float traveled_distance = 0.f;
    for (CyC_INT i = 1; i < m_config.num_waypoints; ++i)
    {
        traveled_distance += distances[i - 1];
        const float t = traveled_distance / total_distance;
        out_waypoints.push_back(spline(t));
    }

    // move the reference waypoints towards the global trajectory
    const float final_gain = 0.50F; // 50%
    const float gain_step = final_gain / (waypoints.size() - 1);

    Eigen::VectorXf gain_vec = trajectory[closest_point].head(3) - state.head(3);
    gain_vec.x() = 0.F; // don't touch the X coordinate

    float gain = 0.F;
    for (auto& point : out_waypoints)
    {
        point += gain_vec * gain;
        gain += gain_step;
    }

    return out_waypoints;
}

CycReferenceSetPoints CPlannerCMNST::compute(
    const std::vector<Eigen::Vector3f>& obstacles,
    const std::vector<Eigen::Vector4f>& global_trajectory,
    const CycState& robot_state,
    float r_safe)
{
    const float longitudinal_step = m_config.longitudinal_distance / m_config.num_waypoints;
    const float vertex_step = 2.F * PI / m_config.num_vertices;

    const auto waypoints = get_reference_waypoints(global_trajectory, robot_state.x_hat);

    CycReferenceSetPoints out;

    for (CyC_INT vertex_idx = 0; vertex_idx < m_config.num_vertices; ++vertex_idx)
    {
        for (CyC_INT layer_idx = 0; layer_idx < m_config.num_layers; ++layer_idx)
        {
            Eigen::VectorXf xvals = Eigen::VectorXf::Zero(m_config.num_waypoints);
            Eigen::VectorXf yvals = Eigen::VectorXf::Zero(m_config.num_waypoints);
            Eigen::VectorXf zvals = Eigen::VectorXf::Zero(m_config.num_waypoints);

            for (CyC_INT wp_idx = 1; wp_idx < m_config.num_waypoints; ++wp_idx)
            {
                const float z = (layer_idx + 1) * m_config.layer_distance + (CyC_rand(m_config.layer_factor) * m_config.layer_distance);
                const float y = 0.F;

                const float alpha = m_config.offset + (vertex_idx * vertex_step) + (CyC_rand(m_config.vertex_factor) * vertex_step);

                xvals[wp_idx] = (wp_idx + 1) * longitudinal_step; // x coordinate
                yvals[wp_idx] = z * sinf(alpha) + y * cosf(alpha) + waypoints[wp_idx].y();
                zvals[wp_idx] = z * cosf(alpha) - y * sinf(alpha) + waypoints[wp_idx].z();
            }

#ifdef HAS_SPLINE
            if (m_config.use_spline)
                out.ref_samples.emplace_back(generate_spline_trajectory(m_config, xvals, yvals, zvals));
            else
#endif // HAS_SPLINE
                out.ref_samples.emplace_back(generate_polynomial_trajectory(m_config, xvals, yvals, zvals));
        }
    }

    if (!out.ref_samples.empty())
        out.ref = out.ref_samples.front(); // TODO

    return out;
}

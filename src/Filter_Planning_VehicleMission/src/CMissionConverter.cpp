// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CMissionConverter.h"
#include "env/CGeolocation.h"

float CMissionConverter::dist(float x1, float y1, float x2, float y2)
{
    return sqrtf(powf(x2 - x1, 2.f) + powf(y2 - y1, 2.f));
}

void CMissionConverter::median_filter(std::vector<Eigen::Vector4f>& points, CyC_UINT kernel_size)
{
    if (kernel_size % 2 == 0)
    {
        spdlog::error("{}: Can not apply median filter with even kernel_size: {}", typeid(*this).name(), kernel_size);
        return;
    }

    for (size_t idx = 0; idx < points.size(); ++idx)
    {
        if ((idx > (kernel_size / 2)) && (idx < (points.size() - (kernel_size / 2))))
        {
            float sum_x = 0;
            float sum_y = 0;
            for (size_t k = (idx - (kernel_size / 2)); k < (idx + (kernel_size / 2)); ++k)
            {
                sum_x += points[k][0];
                sum_y += points[k][1];
            }

            points[idx][0] = sum_x / kernel_size;
            points[idx][1] = sum_y / kernel_size;
        }
    }
}

void CMissionConverter::gps_points2mission(const std::vector<Eigen::Vector3f>& gpsPoints, std::vector<Eigen::Vector4f>& mission_pts)
{
    mission_pts.clear();
    const bool translate_points = true;  // Translate into origin
    const float velocity = 1.2f;  // Reference velocity
    const bool interpolate_points = true;
    const bool filter_points = false;
    const CyC_UINT kernel_size = 7;  // Median filter kernel size

    for (size_t idx = 0; idx < gpsPoints.size(); ++idx)
    {
        // Convert from GPS coordinates to cartesian coordinates
        //const auto pt = CGeolocation::gps2cartesian(gpsPoints[idx][0], gpsPoints[idx][1], gpsPoints[idx][2]);
        const auto pt = CGeolocation::gps2utm(gpsPoints[idx][0], gpsPoints[idx][1], gpsPoints[idx][2]);
        const auto x = pt.x();
        const auto y = pt.y();
        auto yaw = 0.f;
        if (mission_pts.size() > 0)
        {
            const auto prev_x = mission_pts.back()[0];
            const auto prev_y = mission_pts.back()[1];
            const auto prev_angle = mission_pts.back()[3];

            // Estimate yaw angle for each state
            yaw = atan2f(y - prev_y, x - prev_x);

            // Interpolate points in between measurements
            if (interpolate_points)
            {
                const Eigen::Vector4f begin{ prev_x, prev_y, velocity, prev_angle };
                const Eigen::Vector4f end{ x, y, velocity, yaw };
                interpolate_between_points(begin, end, mission_pts);
            }
        }
        else
        {
            yaw = 0.f;
        }

        mission_pts.emplace_back(x, y, velocity, yaw);
    }

    // Translate trajectory points into origin
    if (translate_points && !mission_pts.empty())
    {
        const auto init_x = mission_pts[0][0];
        const auto init_y = mission_pts[0][1];

        for (size_t idx = 0; idx < mission_pts.size(); ++idx)
        {
            mission_pts[idx][0] -= init_x;
            mission_pts[idx][1] -= init_y;
        }
    }

    // Filter data
    if (filter_points)
        median_filter(mission_pts, kernel_size);
}

bool CMissionConverter::gps2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts)
{
    std::vector<Eigen::Vector3f> gpsPoints;
    const float sampling_distance = 0.5f;  // If points will be interpolated, use this distance

    csv_reader.select_cols("lat", "lng", "alt");
    float lat, lng, alt;
    while (csv_reader.read_row(lat, lng, alt))
        gpsPoints.emplace_back(lat, lng, alt);

    gps_points2mission(gpsPoints, _out_ref_path_pts);

    return true;
}

void CMissionConverter::state_points2mission(const std::vector<Eigen::Vector4f>& state_pts, std::vector<Eigen::Vector4f>& mission_pts)
{
    mission_pts.clear();
    // Allow for a limited number of  consecutive points at the same coordinates
    // This is done because the local planner doesn't fare well with 
    // consecutive points with the same x, y values
    CyC_UINT num_consecutive = 0;
    const CyC_UINT num_skip = 5;
    float prev_x = 0, prev_y = 0;

    // Interpolate points, in case of gaps in the trajectory
    const float sampling_distance = 0.1f;  // If points will be interpolated, use this distance
    const bool interpolate_points = true;

    for (const auto& state : state_pts)
    {
        float x, y, v, yaw;
        x = state.x();
        y = state.y();
        v = state.z();
        yaw = state.w();

        if ((prev_x != x) || (prev_y != y))
        {
            num_consecutive = 0;
        }
        else
        {
            num_consecutive++;
        }

        if (num_consecutive < num_skip)
        {
            if (interpolate_points)
            {
                const Eigen::Vector4f begin{ prev_x, prev_y, v, yaw };
                const Eigen::Vector4f end{ x, y, v, yaw };
                interpolate_between_points(begin, end, mission_pts);
            }

            mission_pts.emplace_back(x, y, v, yaw);
            prev_x = x;
            prev_y = y;
        }
    }
}

bool CMissionConverter::state2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts)
{
    std::vector<Eigen::Vector4f> state_pts;

    csv_reader.select_cols("state_variable_0", "state_variable_1", "state_variable_2", "state_variable_3");
    float x, y, v, yaw;
    while (csv_reader.read_row(x, y, v, yaw))
        state_pts.emplace_back(x, y, v, yaw);

    state_points2mission(state_pts, _out_ref_path_pts);

    return true;
}

void CMissionConverter::landmark2mission(const CycLandmark& _landmark, std::vector<Eigen::Vector4f>& _mission_pts)
{
    std::vector<Eigen::Vector2f> waypoints;
    for (const auto& w : _landmark.waypoints)
        waypoints.emplace_back(w.x(), w.y());
    waypoints.emplace_back(_landmark.pose.translation_3x1().x(), _landmark.pose.translation_3x1().y());

    waypoints2mission(waypoints, _mission_pts);
}

void CMissionConverter::waypoints2mission(const std::vector<Eigen::Vector2f>& waypoints, std::vector<Eigen::Vector4f>& mission_pts)
{
    mission_pts.clear();
    const bool interpolate_points = true;
    const float velocity = 1.F;  // Reference velocity
    float yaw = 0.F;       // Default yaw

    for (const auto& w : waypoints)
    {
        float x = w.x(), y = w.y();

        if (mission_pts.size() > 0)
        {
            const auto prev_x = mission_pts.back()[0];
            const auto prev_y = mission_pts.back()[1];

            // Estimate yaw angle for each state
            yaw = atan2f(y - prev_y, x - prev_x);

            // Interpolate points in between measurements
            if (interpolate_points)
            {
                const auto prev_angle = mission_pts.back()[3];
                const Eigen::Vector4f begin{ prev_x, prev_y, velocity, prev_angle };
                const Eigen::Vector4f end{ x, y, velocity, yaw };
                interpolate_between_points(begin, end, mission_pts);
            }
        }

        mission_pts.emplace_back(x, y, velocity, yaw);
    }
}

bool CMissionConverter::waypoints2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts)
{
    std::vector<Eigen::Vector2f> pts;

    csv_reader.select_cols("x", "y", "waypoints");
    float x, y;
    std::string waypoints;
    float x_wpt, y_wpt;
    std::string delimiter_outside = "[";
    std::string delimiter_inside = ";";
    while (csv_reader.read_row(x, y, waypoints))
    {
        // Insert landmark to the reference set points vector
        pts.emplace_back(x, y);

        // Remove first two "["
        waypoints = waypoints.substr(2, waypoints.size() - 1);

        // Replace trailing "]" with delimiter "["
        waypoints[waypoints.size() - 1] = '[';

        // Split waypoints
        unsigned long long pos = 0;
        std::string waypoint;
        while ((pos = waypoints.find(delimiter_outside)) != std::string::npos)
        {
            waypoint = waypoints.substr(0, pos);

            // Replace trailing "]" with delimiter ";"
            waypoint[waypoint.size() - 1] = ';';

            unsigned long long pos2 = 0;
            std::string wpt_element;
            CyC_INT i = 0;
            while ((pos2 = waypoint.find(delimiter_inside)) != std::string::npos)
            {
                wpt_element = waypoint.substr(0, pos2);

                if (i == 0)
                {
                    x_wpt = std::stof(wpt_element);
                }
                if (i == 1)
                {
                    y_wpt = std::stof(wpt_element);
                }
                i++;
                waypoint.erase(0, pos2 + delimiter_inside.length());
            }

            pts.emplace_back(x_wpt, y_wpt);

            waypoints.erase(0, pos + delimiter_outside.length());
        }
    }

    waypoints2mission(pts, _out_ref_path_pts);

    return true;
}

void CMissionConverter::interpolate_between_points(const Eigen::Vector4f& begin, const Eigen::Vector4f& end, std::vector<Eigen::Vector4f>& _in_out_ref_path_pts)
{
    const float sampling_distance = 0.1f;  // If points will be interpolated, use this distance

    const auto prev_x = begin[0];
    const auto prev_y = begin[1];

    const auto x = end[0];
    const auto y = end[1];
    const auto v = end[2];
    const auto yaw = end[3];

    if (!_in_out_ref_path_pts.empty())
    {
#ifdef USE_OLD_METHOD // Uses first degree polynomials. Can't interpolate if slope is infinite
        const float m = (y - prev_y) / (x - prev_x);
        if (abs(x - prev_x) < FLT_EPSILON)
            return;

        const float b = y - m * x;
        const float d = dist(x, y, prev_x, prev_y);
        if (d > sampling_distance)
        {
            const CyC_INT num_interpolate = (CyC_INT)(d / sampling_distance);
            for (CyC_INT interp_idx = 0; interp_idx < num_interpolate; ++interp_idx)
            {
                const float new_x = prev_x + interp_idx * ((x - prev_x) / num_interpolate);
                const float new_y = m * new_x + b;
                const float new_yaw = atan2f(new_y - prev_y, new_x - prev_x);
                _in_out_ref_path_pts.emplace_back(new_x, new_y, v, new_yaw);
            }
        }
#else
        const float d = dist(x, y, prev_x, prev_y);
        const CyC_INT num_interpolate = (CyC_INT)ceil(d / sampling_distance);
        if (num_interpolate > 0)
        {
            const float dx = (x - prev_x) / num_interpolate;
            const float dy = (y - prev_y) / num_interpolate;
            float new_x = prev_x;
            float new_y = prev_y;
            for (CyC_INT interp_idx = 0; interp_idx < num_interpolate; ++interp_idx)
            {
                new_x += dx;
                new_y += dy;

                const float new_yaw = atan2f(new_y - prev_y, new_x - prev_x);
                _in_out_ref_path_pts.emplace_back(new_x, new_y, v, new_yaw);
            }
        }
#endif
    }
}

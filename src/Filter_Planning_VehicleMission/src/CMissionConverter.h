// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include <csv_reader.h>

class CMissionConverter
{
public:
    CMissionConverter() {};
    CMissionConverter(const CMissionConverter&) = default;
    CMissionConverter(CMissionConverter&&) = default;
    CMissionConverter& operator=(const CMissionConverter&) = default;
    CMissionConverter& operator=(CMissionConverter&&) = default;
    ~CMissionConverter() = default;

    bool gps2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts);
    bool state2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts);
    bool waypoints2mission(csv::reader& csv_reader, std::vector<Eigen::Vector4f>& _out_ref_path_pts);

    void gps_points2mission(const std::vector<Eigen::Vector3f>& gpsPoints, std::vector<Eigen::Vector4f>& mission_pts);
    void state_points2mission(const std::vector<Eigen::Vector4f>& statePoints, std::vector<Eigen::Vector4f>& mission_pts);
    void landmark2mission(const CycLandmark& _landmark, std::vector<Eigen::Vector4f>& _mission_pts);
    void waypoints2mission(const std::vector<Eigen::Vector2f>& waypoints, std::vector<Eigen::Vector4f>& mission_pts);
    
    float dist(float x1, float y1, float x2, float y2);
    void median_filter(std::vector<Eigen::Vector4f>& points, CyC_UINT kernel_size);
    void interpolate_between_points(const Eigen::Vector4f& begin, const Eigen::Vector4f& end, std::vector<Eigen::Vector4f>& _in_out_ref_path_pts);
};

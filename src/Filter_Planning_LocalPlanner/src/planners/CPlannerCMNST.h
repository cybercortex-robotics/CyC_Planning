#ifndef CyC_LOCALPLANNER_CMNST_H
#define CyC_LOCALPLANNER_CMNST_H

#include <CyC_TYPES.h>
#include <Eigen/Core>

#ifdef __has_include
#if __has_include(<unsupported/Eigen/Splines>)

#define HAS_SPLINE

#endif // __has_include(<unsupported/Eigen/Splines>)
#endif // __has_include

struct CMNSTConfig
{
    // Number of waypoints
    const CyC_INT num_waypoints = 4;
    // How many layers of lateral points to consider
    CyC_INT num_layers = 3; // [#]
    // How many points on the circle
    CyC_INT num_vertices = 10; // [#]
    // Offset in radians for the first point
    float offset = 0.F; // [rad]
    // Longitudinal distance to generate the points
    float longitudinal_distance = 1.F; // [m]
    // Distance between layers
    float layer_distance = 0.2F; // [m]
    // Layer sampling variance factor
    float layer_factor = 0.F; // [?]
    // Vertices sampling variance factor
    float vertex_factor = 0.F; // [?]
    // Num points to sample from spline
    CyC_INT num_samples = 100; // [#]
    // Use spline, otherwise polynomial
    bool use_spline = false;
};

class CPlannerCMNST
{
public:
    using ParametersType = std::map<std::string, std::string>;

    CPlannerCMNST() = default;
    CPlannerCMNST(const CPlannerCMNST&) = delete;
    CPlannerCMNST(CPlannerCMNST&&) = delete;
    CPlannerCMNST& operator=(const CPlannerCMNST&) = delete;
    CPlannerCMNST& operator=(CPlannerCMNST&&) = delete;
    ~CPlannerCMNST() = default;

    void loadConfiguration(const ParametersType& parameters);

    CycReferenceSetPoints compute(
        const std::vector<Eigen::Vector3f>& obstacles,
        const std::vector<Eigen::Vector4f>& global_trajectory,
        const CycState& robot_state,
        float r_safe);

private:
    CycTrajectory get_reference_waypoints(
        const std::vector<Eigen::Vector4f>& trajectory,
        const Eigen::VectorXf& state);

    CMNSTConfig m_config;
    size_t m_last_closest_point_idx = 0;
};

#endif // CyC_LOCALPLANNER_CMNST_H

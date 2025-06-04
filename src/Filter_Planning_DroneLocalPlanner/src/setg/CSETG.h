#ifndef CyC_SETG_H_
#define CyC_SETG_H_

#include <vector>
#include <Eigen/Core>
#include <CyC_TYPES.h>

namespace util
{
    template <typename... Args>
    Eigen::VectorXf makeVector(Args... v)
    {
        Eigen::VectorXf vec(sizeof...(Args));
        size_t i = 0;

        auto assign = [&vec, &i](auto v) { vec[i] = v; i++; };
        bool unused[] = { (assign(v), true)... };

        return vec;
    }
}

class CSETG
{
public:
    CSETG() = default;
    CSETG(const CSETG&) = default;
    CSETG(CSETG&&) = default;
    CSETG& operator=(const CSETG&) = default;
    CSETG& operator=(CSETG&&) = default;
    ~CSETG() = default;

    std::vector<Eigen::VectorXf> computePath(const std::vector<Eigen::VectorXf>& waypoints, const std::vector<Eigen::VectorXf>& obstacles, float r_safe);

private:
    struct STangentLine
    {
        float slope = 0.F;
        float intercept = 0.F;
        Eigen::VectorXf point = Eigen::VectorXf::Zero(2);
    };

    static Eigen::MatrixXf getLinePoints(const Eigen::VectorXf& p1, const Eigen::VectorXf& p2);
    static bool isInsideObstacle(const Eigen::MatrixXf& points, const Eigen::VectorXf& obstacle, float r_safe);
    static size_t findFirstIntersection(const Eigen::VectorXf& p1, const Eigen::VectorXf& p2, const std::vector<Eigen::VectorXf>& obstacles, float r_safe);
    static Eigen::VectorXf findIntersectionPoint(const STangentLine& line1, const STangentLine& line2);
    static std::vector<STangentLine> findTangentLines(const Eigen::VectorXf& center, const Eigen::VectorXf& semi_axes, float theta, const Eigen::VectorXf& reference_point);

    std::vector<Eigen::VectorXf> Pa; // waypoints set
    std::vector<Eigen::VectorXf> Ca; // candidate waypoints
    std::unordered_set<size_t> Ba; // obstacles avoided

    static const size_t NOT_FOUND_INDEX = std::numeric_limits<size_t>::max();
};

#endif CyC_SETG_H_

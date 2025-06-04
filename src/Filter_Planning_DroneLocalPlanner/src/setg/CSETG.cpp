#include "CSETG.h"
#include <numeric>
#include <algorithm>
#include <Eigen/Dense>

template <typename T>
std::vector<size_t> argsort(const std::vector<T>& v)
{
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    std::stable_sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}

template <typename T>
size_t argmin(const std::vector<T>& v)
{
    size_t idx = 0;
    for (size_t i = 0; i < v.size(); ++i)
    {
        if (v[i] < v[idx])
        {
            idx = i;
        }
    }

    return idx;
}

template <class _ContainerType>
class stack_push_iterator // wrap pushes to stack as output iterator
{
public:
    using iterator_category = std::output_iterator_tag;
    using value_type = void;
    using pointer = void;
    using reference = void;

    using container_type = std::stack<_ContainerType>;
    using difference_type = void;

    explicit stack_push_iterator(std::stack<_ContainerType>& _Cont) noexcept
        : container(std::addressof(_Cont)) {}

    stack_push_iterator& operator=(const typename container_type::value_type& _Val)
    {
        container->push(_Val);
        return *this;
    }

    stack_push_iterator& operator=(typename container_type::value_type&& _Val)
    {
        container->push(std::move(_Val));
        return *this;
    }

    stack_push_iterator& operator*() noexcept
    {
        return *this;
    }

    stack_push_iterator& operator++() noexcept
    {
        return *this;
    }

    stack_push_iterator operator++(int) noexcept
    {
        return *this;
    }

private:
    container_type* container = nullptr;
};


std::vector<Eigen::VectorXf> CSETG::computePath(const std::vector<Eigen::VectorXf>& waypoints, const std::vector<Eigen::VectorXf>& obstacles, float r_safe)
{
    if ((waypoints.size() < 2) || obstacles.empty())
    {
        return waypoints;
    }

    std::vector<Eigen::VectorXf> path;
    path.push_back(waypoints[0]);

    const size_t N = waypoints.size() - 1;
    for (size_t i = 0; i < N; ++i)
    {
        Pa.clear();
        Ca.clear();
        Ba.clear();

        Pa.push_back(waypoints[i]);
        Ca.push_back(waypoints[i + 1]);

        while (!Ca.empty())
        {
            const Eigen::VectorXf& O = Pa.back();
            const Eigen::VectorXf& D = Ca.back();

            size_t colliding_idx = findFirstIntersection(O, D, obstacles, r_safe);
            if (colliding_idx == NOT_FOUND_INDEX)
            {
                Pa.push_back(D);
                Ca.pop_back();
            }
            else
            {
                std::stack<Eigen::VectorXf> obstacle_stack;
                obstacle_stack.push(obstacles[colliding_idx]);

                std::vector<bool> obstacles_considered(obstacles.size());
                std::fill(obstacles_considered.begin(), obstacles_considered.end(), false);

                std::vector<Eigen::VectorXf> candidates;
                while (candidates.size() < 2)
                {
                    const Eigen::VectorXf& obs = obstacle_stack.top();

                    const Eigen::VectorXf center = obs.head(3);
                    const Eigen::VectorXf semi_axes = obs.segment(3, 2).array() + (r_safe * 1.1F); // 10% tolerance?
                    const float theta = obs.tail(1)[0];

                    const auto tangents_O = findTangentLines(center, semi_axes, theta, O);
                    const auto tangents_D = findTangentLines(center, semi_axes, theta, D);

                    if (tangents_O.empty() || tangents_D.empty())
                    {
                        return {};
                    }

                    // How to determine the tangents on the same side ? ?
                    // Current method : determine which two tangent points are the closest
                    const std::vector<std::vector<STangentLine>> combinations = {
                        { tangents_O.front(), tangents_D.front() },
                        { tangents_O.front(), tangents_D.back() },
                        { tangents_O.back(), tangents_D.front() },
                        { tangents_O.back(), tangents_D.back() }
                    };

                    std::vector<float> costs;
                    for (const auto& combination : combinations)
                    {
                        costs.push_back((combination[0].point - combination[1].point).norm());
                    }

                    const auto indices = argsort(costs);
                    const auto& combo1 = combinations[indices[0]];
                    const auto& combo2 = combinations[indices[1]];

                    const Eigen::VectorXf candidate1 = findIntersectionPoint(combo1[0], combo1[1]);
                    const Eigen::VectorXf candidate2 = findIntersectionPoint(combo2[0], combo2[1]);

                    std::vector<Eigen::VectorXf> tmp_candidates = { candidate1, candidate2 };
                    std::vector<bool> in_obstacle = { false, false };
                    std::vector<Eigen::VectorXf> new_obstacles;

                    for (size_t j = 0; j < obstacles.size(); ++j)
                    {
                        const auto& obstacle = obstacles[j];
                        for (size_t i = 0; i < tmp_candidates.size(); ++i)
                        {
                            const auto& candidate = tmp_candidates[i];
                            if (isInsideObstacle(candidate.matrix(), obstacle, r_safe))
                            {
                                if (!obstacles_considered[j])
                                {
                                    obstacles_considered[j] = true;
                                    new_obstacles.push_back(obstacle);
                                }

                                in_obstacle[i] = true;
                            }
                        }
                    }

                    for (size_t i = 0; i < tmp_candidates.size(); ++i)
                    {
                        const auto in_obs = in_obstacle[i];
                        const auto& candidate = tmp_candidates[i];

                        if (!in_obs)
                        {
                            candidates.push_back(candidate);
                        }
                    }

                    obstacle_stack.pop();
                    std::copy(new_obstacles.begin(), new_obstacles.end(), stack_push_iterator(obstacle_stack));
                }

                // How to choose which candidate point is the best?
                // Current: based on summed distances
                const std::vector<float> distances = {
                    (O.head(2) - candidates.front()).norm() + (D.head(2) - candidates.front()).norm(),
                    (O.head(2) - candidates.back()).norm() + (D.head(2) - candidates.back()).norm()
                };

                const auto pick = argmin(distances);
                Eigen::VectorXf T = (pick == 0) ? candidates.front() : candidates.back();

                // interpolate Z
                // https://math.stackexchange.com/questions/105400/linear-interpolation-in-3-dimensions
                const Eigen::VectorXf v = D - O;
                const float d = (T - O.head(2)).norm();
                const Eigen::VectorXf M = O + (d / v.norm()) * v;

                T.conservativeResize(T.size() + 1);
                T.z() = M.z();

                Ba.insert(colliding_idx);

                colliding_idx = findFirstIntersection(O, T, obstacles, r_safe);
                if (colliding_idx == NOT_FOUND_INDEX)
                {
                    Pa.push_back(T);
                }
                else
                {
                    Ca.push_back(T);
                }
            }
        }

        std::copy(Pa.begin() + 1, Pa.end(), std::back_inserter(path));
    }

    return path;
}

Eigen::MatrixXf CSETG::getLinePoints(const Eigen::VectorXf& p1, const Eigen::VectorXf& p2)
{
    const float step_size = 0.1F;
    const CyC_INT N = (CyC_INT)ceil((p1.head<2>() - p2.head<2>()).norm() / step_size);

    Eigen::MatrixXf points = Eigen::MatrixXf::Zero(2, N);
    points.row(0) = Eigen::VectorXf::LinSpaced(N, p1.x(), p2.x());
    points.row(1) = Eigen::VectorXf::LinSpaced(N, p1.y(), p2.y());

    return points;
}

bool CSETG::isInsideObstacle(const Eigen::MatrixXf& points, const Eigen::VectorXf& obstacle, float r_safe)
{
    const float x = obstacle[0];
    const float y = obstacle[1];
    const float z = obstacle[2];
    const float a = obstacle[3];
    const float b = obstacle[4];
    const float theta = obstacle[5];

    const Eigen::VectorXf t = obstacle.head(2);

    Eigen::VectorXf R1 = Eigen::VectorXf::Zero(2);
    Eigen::VectorXf R2 = Eigen::VectorXf::Zero(2);
    R1 << cosf(theta), sinf(theta);
    R2 << -sinf(theta), cosf(theta);

    const Eigen::MatrixXf pts = points.colwise() - t;

    const Eigen::MatrixXf rotatedTerm1 = pts.array().colwise() * R1.array();
    const Eigen::MatrixXf rotatedTerm2 = pts.array().colwise() * R2.array();

    const Eigen::VectorXf term1 = rotatedTerm1.colwise().sum().array().pow(2.F) / powf(a + r_safe, 2.F);
    const Eigen::VectorXf term2 = rotatedTerm2.colwise().sum().array().pow(2.F) / powf(b + r_safe, 2.F);

    const Eigen::VectorXf distances = term1 + term2;

    const bool res = (distances.array() < 1.F).any();
    return res;
}

size_t CSETG::findFirstIntersection(const Eigen::VectorXf& p1, const Eigen::VectorXf& p2, const std::vector<Eigen::VectorXf>& obstacles, float r_safe)
{
    // TODO: BUG!!!!!!!!!!!!!!
    // As of now, this does not find the first obstacle the line intersects physically, but first in the list of obstacles!!
    const Eigen::MatrixXf points = getLinePoints(p1, p2);

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        if (isInsideObstacle(points, obstacles[i], r_safe))
        {
            return i;
        }
    }

    return NOT_FOUND_INDEX;
}

Eigen::VectorXf CSETG::findIntersectionPoint(const STangentLine& line1, const STangentLine& line2)
{
    Eigen::VectorXf point = Eigen::VectorXf::Zero(2);

    point.x() = (line2.intercept - line1.intercept) / (line1.slope - line2.slope);
    point.y() = line1.slope * point.x() + line1.intercept;

    return point;
}

// https://stackoverflow.com/questions/70776421/tangent-lines-to-a-rotated-ellipse-from-a-point
std::vector<CSETG::STangentLine> CSETG::findTangentLines(const Eigen::VectorXf& center, const Eigen::VectorXf& semi_axes, float rot, const Eigen::VectorXf& reference_point)
{
    const float x0 = center.x();
    const float y0 = center.y();

    const float a = semi_axes.x();
    const float b = semi_axes.y();

    const float s = sinf(rot);
    const float c = cosf(rot);

    const float p0 = reference_point.x();
    const float q0 = reference_point.y();

    const float A = y0 - q0;
    const float D = x0 - p0;
    const float B = a * s;
    const float E = a * c;
    const float C = b * c;
    const float F = -b * s;

    const float denominator = sqrtf(powf(C * D - A * F, 2.F) + powf(A * E - B * D, 2.F));
    if (fabsf((B * F - C * E) / denominator) > 1.F)
    {
        spdlog::error("SETG: Reference point lies inside the ellipse!");
        return {};
    }

    const float beta = atan2f(
        (C * D - A * F) / denominator,
        (A * E - B * D) / denominator
    );

    const std::vector<float> theta = {
        -beta + asinf((B * F - C * E) / denominator),
        -beta - asinf((B * F - C * E) / denominator) + PI
    };

    std::vector<STangentLine> lines(2);
    for (size_t i = 0; i < lines.size(); ++i)
    {
        const auto t = theta[i];
        auto& line = lines[i];

        line.point.x() = x0 + E * cosf(t) + F * sinf(t);
        line.point.y() = y0 + B * cosf(t) + C * sinf(t);

        line.slope = (line.point.y() - q0) / (line.point.x() - p0);
        line.intercept = line.point.y() - line.slope * line.point.x();
    }

    return lines;
}

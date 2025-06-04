#include "setg/CSETG.h"
#include <opencv2/opencv.hpp>

const int WIDTH = 500;
const int HEIGHT = 500;

const cv::Size size{ WIDTH, HEIGHT };
const cv::Point center = size / 2;

std::vector<Eigen::VectorXf> obstacles;

void onMouseClick(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        obstacles.push_back(util::makeVector(x - center.x, y - center.y, 0.F, 20.F, 40.F, 45.F * DEG2RAD));
    }
}

int main()
{
    obstacles.push_back(util::makeVector(0.F, 0.F, 0.F, 20.F, 40.F, 45.F * DEG2RAD));
    obstacles.push_back(util::makeVector(150.F, 0.F, 0.F, 20.F, 40.F, 45.F * DEG2RAD));

    std::vector<Eigen::VectorXf> reference_path;
    reference_path.push_back(util::makeVector(-150.F, -150.F, 0.F));
    reference_path.push_back(util::makeVector(150.F, 150.F, 0.F));
    reference_path.push_back(util::makeVector(150.F, -150.F, 0.F));

    cv::namedWindow("image", 1);
    cv::setMouseCallback("image", onMouseClick, NULL);

    while (true)
    {
        cv::Mat image = cv::Mat::ones(size, CV_8UC3);
        image.setTo(cv::Scalar(255, 255, 255));

        for (const auto& obs : obstacles)
        {
            cv::ellipse(image, cv::Point(obs[0], obs[1]) + center, cv::Size(obs[3], obs[4]), obs[5] * RAD2DEG, 0, 360, cv::Scalar(0, 0, 255), -1);
        }

        CSETG planner;
        auto points = planner.computePath(reference_path, obstacles, 1.F);

        if (!points.empty())
        {
            for (size_t i = 0; i < points.size() - 1; i++)
            {
                const auto& p1 = points[i];
                const auto& p2 = points[i + 1];

                cv::line(image, cv::Point(p1.x(), p1.y()) + center, cv::Point(p2.x(), p2.y()) + center, cv::Scalar(255, 0, 0), 2);
                cv::circle(image, cv::Point(p1.x(), p1.y()) + center, 3, cv::Scalar(0, 0, 0), -1);
                cv::circle(image, cv::Point(p2.x(), p2.y()) + center, 3, cv::Scalar(0, 0, 0), -1);
            }
        }

        if (!reference_path.empty())
        {
            for (size_t i = 0; i < reference_path.size() - 1; i++)
            {
                const auto& p1 = reference_path[i];
                const auto& p2 = reference_path[i + 1];

                cv::line(image, cv::Point(p1.x(), p1.y()) + center, cv::Point(p2.x(), p2.y()) + center, cv::Scalar(0, 255, 0), 2);
                cv::circle(image, cv::Point(p1.x(), p1.y()) + center, 3, cv::Scalar(0, 0, 0), -1);
                cv::circle(image, cv::Point(p2.x(), p2.y()) + center, 3, cv::Scalar(0, 0, 0), -1);
            }
        }

        cv::imshow("image", image);
        if ((cv::waitKey(30) & 0xFF) == 27)
            break;
    }

    return 0;
}

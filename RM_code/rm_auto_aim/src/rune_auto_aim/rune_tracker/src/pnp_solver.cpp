#include "rune_tracker/pnp_solver.hpp"

namespace rune {
PnPSolver::PnPSolver(
    const std::vector<double>& camera_matrix,
    const std::vector<double>& distortion_coefficients
):
    camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
    distortion_coefficients_(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_coefficients.data())).clone()) {}

std::vector<cv::Point3d>
PnPSolver::GeneratePw(double outerwidth, double insidewidth, double height) {
    return {
        { -1 * outerwidth / 2, height / 2, 0 },
        { -1 * insidewidth / 2, -1 * height / 2, 0 },
        { insidewidth / 2, -1 * height / 2, 0 },
        { outerwidth / 2, height / 2, 0 },
    };
}

/**
         * @brief 使用PnP根据像素坐标获取平移向量并转换成Eigen::Vector3d
         *
         * @param rune 符叶像素坐标
         * @return tvec 平移向量，相机坐标系下的坐标
         */
bool PnPSolver::SolvePnP(std::vector<cv::Point2d>& rune, cv::Mat& rvec, cv::Mat& tvec, bool iterate) {
    // Solve pnp
    bool ret = cv::solvePnP(
        GeneratePw(RUNE_PNP_OUTER_LIGHTBAR_WIDTH, RUNE_PNP_INSIDE_LIGHTBAR_WIDTH, RUNE_PNP_RADIUS),
        rune,
        camera_matrix_,
        distortion_coefficients_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (iterate) {
        ret = cv::solvePnP(
            GeneratePw(RUNE_PNP_OUTER_LIGHTBAR_WIDTH, RUNE_PNP_INSIDE_LIGHTBAR_WIDTH, RUNE_PNP_RADIUS),
            rune,
            camera_matrix_,
            distortion_coefficients_,
            rvec,
            tvec,
            true,
            cv::SOLVEPNP_IPPE
        );
    }
    return ret;
}

float PnPSolver::CalculateDistanceToCenter(const cv::Point2f& image_point) {
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

} // namespace rune

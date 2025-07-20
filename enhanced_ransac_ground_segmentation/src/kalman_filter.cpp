#include "kalman_filter.h"
#include <vector>

KalmanFilter::KalmanFilter() : is_initialized_(false)
{
    // For a simple "constant plane" model, the state transition and
    // measurement matrices are both identity matrices.
    // This means we expect the plane to be the same in the next frame,
    // and we are directly measuring the plane coefficients.
    F_ = Eigen::Matrix4f::Identity();
    H_ = Eigen::Matrix4f::Identity();
}

void KalmanFilter::init(const std::vector<float>& initial_coeffs,
                        double initial_covariance,
                        double process_noise,
                        double measurement_noise)
{
    if (initial_coeffs.size() != 4) {
        // Handle error: cannot initialize with incorrect dimensions
        return;
    }

    // Set the initial state vector
    x_ << initial_coeffs[0], initial_coeffs[1], initial_coeffs[2], initial_coeffs[3];

    // Set the initial uncertainty (covariance)
    P_ = Eigen::Matrix4f::Identity() * initial_covariance;

    // Set the noise matrices based on configuration
    Q_ = Eigen::Matrix4f::Identity() * process_noise;
    R_ = Eigen::Matrix4f::Identity() * measurement_noise;

    is_initialized_ = true;
}

void KalmanFilter::predict()
{
    if (!is_initialized_) {
        return;
    }

    // Predict the state
    // x_pred = F * x_
    x_ = F_ * x_;

    // Predict the state covariance
    // P_pred = F * P * F^T + Q
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const std::vector<float>& measurement)
{
    if (!is_initialized_ || measurement.size() != 4) {
        return;
    }

    // Convert measurement vector to Eigen vector
    Eigen::Vector4f z;
    z << measurement[0], measurement[1], measurement[2], measurement[3];

    // --- Kalman Filter Update Equations ---

    // Innovation (measurement residual)
    // y = z - H * x
    Eigen::Vector4f y = z - H_ * x_;

    // Innovation covariance
    // S = H * P * H^T + R
    Eigen::Matrix4f S = H_ * P_ * H_.transpose() + R_;

    // Kalman Gain
    // K = P * H^T * S^-1
    Eigen::Matrix4f K = P_ * H_.transpose() * S.inverse();

    // Updated state estimate
    // x_new = x + K * y
    x_ = x_ + (K * y);

    // Updated state covariance
    // P_new = (I - K * H) * P
    long size = x_.size();
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    P_ = (I - K * H_) * P_;
}

std::vector<float> KalmanFilter::getState() const
{
    if (!is_initialized_) {
        return {};
    }
    // Convert Eigen vector back to std::vector
    std::vector<float> state_vec(x_.data(), x_.data() + x_.size());
    return state_vec;
}

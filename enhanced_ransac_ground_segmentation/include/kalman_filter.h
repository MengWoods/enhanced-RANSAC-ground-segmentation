#pragma once

#include <Eigen/Dense>
#include <vector>

/**
 * @class KalmanFilter
 * @brief A simple 4D Kalman Filter for tracking plane coefficients [a, b, c, d].
 *
 * This filter assumes a constant velocity model (i.e., the plane is expected
 * to be similar from one frame to the next). It uses the Eigen library for
 * all linear algebra operations.
 */
class KalmanFilter
{
public:
    /**
     * @brief Constructor.
     */
    KalmanFilter();

    /**
     * @brief Initializes the filter with the first measurement.
     * @param initial_coeffs The first set of measured plane coefficients.
     * @param initial_covariance The initial uncertainty of the estimate.
     * @param process_noise The uncertainty in the plane's motion model.
     * @param measurement_noise The uncertainty of the RANSAC measurement.
     */
    void init(const std::vector<float>& initial_coeffs,
              double initial_covariance,
              double process_noise,
              double measurement_noise);

    /**
     * @brief Predicts the state for the next time step.
     */
    void predict();

    /**
     * @brief Updates the state estimate with a new measurement.
     * @param measurement The new plane coefficients from RANSAC.
     */
    void update(const std::vector<float>& measurement);

    /**
     * @brief Gets the current filtered state of the plane coefficients.
     * @return A vector containing the estimated [a, b, c, d].
     */
    std::vector<float> getState() const;

    /**
     * @brief Checks if the filter has been initialized.
     * @return True if initialized, false otherwise.
     */
    bool isInitialized() const { return is_initialized_; }

private:
    // Flag to check if the filter has been initialized
    bool is_initialized_;

    // State vector [a, b, c, d]
    Eigen::Vector4f x_;

    // State covariance matrix (P) - represents the uncertainty of the state
    Eigen::Matrix4f P_;

    // State transition matrix (F) - defines how the state evolves
    Eigen::Matrix4f F_;

    // Process noise covariance matrix (Q) - uncertainty in the motion model
    Eigen::Matrix4f Q_;

    // Measurement matrix (H) - maps the state to the measurement space
    Eigen::Matrix4f H_;

    // Measurement noise covariance matrix (R) - uncertainty of the sensor
    Eigen::Matrix4f R_;
};

#ifndef UKF_PLUS_H
#define UKF_PLUS_H
#include "filter.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class UKF_PLUS: public Filter {
public:
    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* Sigma point spreading parameter
    double lambda_;

    UKF_PLUS();
    UKF_PLUS(bool verbose, bool use_laser, bool use_radar, double std_a, double std_yawdd);

    virtual ~UKF_PLUS();

    void Init(bool verbose = false, bool use_laser = true, bool use_radar = true, double std_a = 2.0, double std_yawdd = 0.5);

    void Clear();
    /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
    void Prediction(double delta_t);

    /**
   * Updates the state and the state covariance matrix using a laser measurement.
   * This function just uses the simple linear Kalman filter equations becasuse
   * the measurement equations are linear (we measure the position states directly).
   * @param {MeasurementPackage} meas_package
   */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
   * Updates the state and the state covariance matrix using a laser measurement
   * by applying the unscented transform instead of the linear kalman filter equations.
   * @param {MeasurementPackage} meas_package
   */
    void UpdateLidarUnscented(MeasurementPackage meas_package);
    /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
    void UpdateRadar(MeasurementPackage meas_package);

    void write_vec(const vector<double>& vec);
};

#endif /* UKF_PLUS_H */

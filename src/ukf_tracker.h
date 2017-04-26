#ifndef UKF_TRACKER_H_
#define UKF_TRACKER_H_

#include "Eigen/Dense"

class UkfTracker {
public:
  // Contains the sensor measurement data: timestamp, sensor type and value
  struct Measurement {
    // Indicates the sensor type
    enum class SensorType {
      kLidar,
      kRadar
    };

    long long timestamp;
    SensorType sensor_type;
    Eigen::VectorXf value;

    Measurement() : timestamp(0), sensor_type(SensorType::kLidar) { }
  };

  // Contains the estimate data: state and NIS
  struct Estimate {
    Eigen::VectorXf x;
    float nis;
    Estimate() : nis(0) { }
    Estimate(Eigen::VectorXf in_x, float in_nis) : x(in_x), nis(in_nis) { }
  };

  // Ctor
  UkfTracker();

  // Dtor
  virtual ~UkfTracker() { }

  // Processes a single measurement and returns the estimate at x+1
  // @param[in] measurement  The measurement data
  // @return Estimate
  Estimate operator()(const Measurement& measurement);

private:
  // Indicates if the object has been initialized
  bool is_initialized_;

  // Previous timestamp
  long previous_timestamp_;

  // State vector
  Eigen::VectorXf x_;

  // State covariance matrix
  Eigen::MatrixXf p_;

  // Predicts the state and the state covariance using the process model
  // @param[in] dt  Time between k and k+1 in s
  // @param[out] Predicted sigma points
  void Predict(float dt, Eigen::MatrixXf& x_sig_pred);

  // Updates the state by using Unscented Kalman Filter equations
  // @param[in] x_sig_pred  Predicted sigma points
  // @param[in] z           Measurement data
  // @param[in] z_sig       Sigma points in measurement space
  // @param[in] z_pred      Predicted measurement mean
  // @param[in] s           Predicted measurement covariance
  // @return NIS
  float Update(const Eigen::MatrixXf& x_sig_pred,
               const Eigen::VectorXf& z,
               const Eigen::MatrixXf& z_sig,
               const Eigen::VectorXf& z_pred,
               const Eigen::MatrixXf& s);
};

#endif // UKF_TRACKER_H_

#include "ukf_tracker.h"
#include <iostream>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// State dimension
const unsigned short kNx = 5;

// Number of sigma points
const unsigned short kNsigX = 2 * kNx + 1;

// Sigma point spreading parameter
const float kLambdaX = 3 - kNx;

// Augmented state dimension
const unsigned short kNaug = 7;

// Number of augmented sigma points
const unsigned short kNsigAug = 2 * kNaug + 1;

// Augmented sigma point spreading parameter
const float kLambdaAug = 3 - kNaug;

// Process noise standard deviation longitudinal acceleration in m/s^2
// TODO: adjust
const float kStdA = 1.2;

// Process noise standard deviation yaw acceleration in rad/s^2
// TODO: adjust
const float kStdYaw = .6;

// Augmented covariance dimension
const unsigned short kNq = 2;

// Augmented covariance matrix
const Eigen::MatrixXf kQ((Eigen::MatrixXf(kNq, kNq)
  << kStdA * kStdA, 0,
     0, kStdYaw * kStdYaw).finished());

const float kEpsilon = 1e-6;

// Augmented weights
Eigen::VectorXf CreateWeights(unsigned short n_points) {
  Eigen::VectorXf weights(2 * n_points + 1);
  float base_weight = 1. / 3;
  weights(0) = base_weight * (3 - n_points);
  weights.tail(2 * n_points).setConstant(base_weight / 2);
  return weights;
}
const Eigen::MatrixXf kAugWeights = CreateWeights(kNaug);

// Measurement dimension: lidar can measure px and py
const unsigned short kNzLidar = 2;

// Lidar measurement noise standard deviation position1 in m
const float kStdPx = 0.15;

// Lidar measurement noise standard deviation position2 in m
const float kStdPy = 0.15;

// Lidar mesurement noise covariance
Eigen::MatrixXf kRlidar((Eigen::MatrixXf(kNzLidar, kNzLidar)
  << kStdPx * kStdPx, 0,
     0, kStdPy * kStdPy).finished());

// Measurement dimension: radar can measure rho, phi, and rho dot
const unsigned short kNzRadar = 3;

// Radar measurement noise standard deviation radius in m
const float kStdRho = 0.3;

// Radar measurement noise standard deviation angle in rad
const float kStdPhi = 0.03;

// Radar measurement noise standard deviation radius change in m/s
const float kStdRhoDot = 0.3;

// Radar mesurement noise covariance
Eigen::MatrixXf kRradar((Eigen::MatrixXf(kNzRadar, kNzRadar)
  << kStdRho * kStdRho, 0, 0,
     0, kStdPhi * kStdPhi, 0,
     0, 0, kStdRhoDot * kStdRhoDot).finished());

// Initial state
// TODO: Try to do better initialization (lesson 7.32)
//       We could assume the average speed, yaw and yaw rate  of the bicycle 
const Eigen::MatrixXf kXinitial((Eigen::VectorXf(kNx)
  << 0, 0, 0, 0, 0).finished());

// Initial state covariance matrix
// TODO: Try to do better initialization (lesson 7.32)
const Eigen::MatrixXf kPinitial((Eigen::MatrixXf(kNx, kNx)
  << 1, 0, 0, 0, 0,
     0, 1, 0, 0, 0,
     0, 0, 1, 0, 0,
     0, 0, 0, 1, 0,
     0, 0, 0, 0, 1).finished());


// Local Helper-Functions
// -----------------------------------------------------------------------------
Eigen::VectorXf ConvertPolarToCartesian(const Eigen::VectorXf& polar) {
  auto rho = polar(0);
  auto phi = polar(1);

  Eigen::VectorXf cartesian(kNx);
  cartesian << rho * std::cos(phi), rho * std::sin(phi), 0, 0, 0;

  return cartesian;
}

void PredictLidarMeasurement(const Eigen::MatrixXf& x_sig_pred,
                             Eigen::MatrixXf& z_sig,
                             Eigen::VectorXf& z_pred,
                             Eigen::MatrixXf& s) {
  // Create matrix for sigma points in measurement space
  z_sig = Eigen::MatrixXf(kNzLidar, kNsigAug);
  // Transform sigma points into measurement space
  for (auto i = 0; i < kNsigAug; ++i) {
    z_sig.col(i)(0) = x_sig_pred.col(i)(0);
    z_sig.col(i)(1) = x_sig_pred.col(i)(1);
  }
  // Calculate mean predicted measurement
  // TODO: remove the explicit definition
  z_pred = Eigen::VectorXf(kNzLidar);
  z_pred = z_sig * kAugWeights;
  // Calculate measurement covariance matrix S
  Eigen::MatrixXf diff(kNzLidar, kNsigAug);
  diff = z_sig.colwise() - z_pred;
  // TODO: remove the explicit definition
  s = Eigen::MatrixXf(kNzLidar, kNzLidar);
  s = diff * kAugWeights.asDiagonal() * diff.transpose() + kRlidar;
}

void PredictRadarMeasurement(const Eigen::MatrixXf& x_sig_pred,
                             Eigen::MatrixXf& z_sig,
                             Eigen::VectorXf& z_pred,
                             Eigen::MatrixXf& s) {
  // Create matrix for sigma points in measurement space
  z_sig = Eigen::MatrixXf(kNzRadar, kNsigAug);
  // Transform sigma points into measurement space
  for (auto i = 0; i < kNsigAug; ++i) {
    // TODO: fix indices
    float px = x_sig_pred.col(i)(0);
    float py = x_sig_pred.col(i)(1);
    float v = x_sig_pred.col(i)(2);
    float psi = x_sig_pred.col(i)(3);
//    std::cout << "px " << px << ", py " << py << std::endl;
    float sqrt_of_sum_of_squares = std::sqrt(px * px + py * py);
    z_sig.col(i)(0) = sqrt_of_sum_of_squares;
    // TODO: handle division by zero
    z_sig.col(i)(1) = std::atan2(py, px);
    z_sig.col(i)(2) = v * (px * std::cos(psi) + py * std::sin(psi))
                      / sqrt_of_sum_of_squares;
  }
  // Calculate mean predicted measurement
  // TODO: remove the explicit definition
  z_pred = Eigen::VectorXf(kNzRadar);
  z_pred = z_sig * kAugWeights;
  // Calculate measurement covariance matrix S
  Eigen::MatrixXf diff(kNzRadar, kNsigAug);
  diff = z_sig.colwise() - z_pred;
  // TODO: remove the explicit definition
  s = Eigen::MatrixXf(kNzRadar, kNzRadar);
  s = diff * kAugWeights.asDiagonal() * diff.transpose() + kRradar;
}
/*
Eigen::VectorXf ConvertCartesianToPolar(const Eigen::VectorXf& cartesian) {
  auto px = cartesian(0);
  auto py = cartesian(1);
  auto vx = cartesian(2);
  auto vy = cartesian(3);

  Eigen::VectorXf polar(Eigen::VectorXf::Zero(3));

  if (std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon) {
    return polar;
  }

  auto sqrt_of_sum_of_squares = std::sqrt(px * px + py * py);

  polar(0) = sqrt_of_sum_of_squares;;
  polar(1) = std::atan2(py, px);
  polar(2) = (px * vx + py * vy) / sqrt_of_sum_of_squares;

  return polar;
}
*/

} // namespace

// Public Members
// -----------------------------------------------------------------------------

UkfTracker::UkfTracker()
  : is_initialized_(false),
    previous_timestamp_(-1),
    x_(kXinitial),
    p_(kPinitial),
    nis_(0) { }

UkfTracker::Estimate UkfTracker::operator()(const Measurement& measurement) {
  if (!is_initialized_) {
//    std::cout << "Initializing..." << std::endl;
    // Initialize the state with the first measurement
    if (measurement.sensor_type == Measurement::SensorType::kRadar) {
      x_ = ConvertPolarToCartesian(measurement.value);
      // TODO: initialize P
    }
    else {
      x_ << measurement.value(0), measurement.value(1), 0, 0, 0;
      // TODO: initialize P
    }
    // Initialize the previous timestamp with the first measurement
    previous_timestamp_ = measurement.timestamp;
    is_initialized_ = true;

    std::cout << "x0" << std::endl << x_ << std::endl;
    std::cout << "P0" << std::endl << p_ << std::endl;
    return Estimate(x_, nis_);
  }

  // Time elapsed between the current and previous measurements in seconds
  float dt = (measurement.timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement.timestamp;

  // Predict state mean and covariance
  Eigen::MatrixXf x_sig_pred(kNx, kNsigAug);
//  std::cout << "Predicting state..." << std::endl;
  Predict(dt, x_sig_pred);
//  std::cout << "Predicted x" << std::endl << x_ << std::endl;
//  std::cout << "Predicted P" << std::endl << p_ << std::endl;

  // Predict measurement mean and covariance
  Eigen::MatrixXf z_sig;
  Eigen::VectorXf z_pred;
  Eigen::MatrixXf s;
//  std::cout << "Predicting measurement..." << std::endl;
  if (measurement.sensor_type == Measurement::SensorType::kRadar) {
    PredictRadarMeasurement(x_sig_pred, z_sig, z_pred, s);
  } else {
    PredictLidarMeasurement(x_sig_pred, z_sig, z_pred, s);
  }

  // Update state mean and covariance
//  std::cout << "Updating..." << std::endl;
  Update(measurement.value, x_sig_pred, z_sig, z_pred, s);
  std::cout << "Updated x" << std::endl << x_ << std::endl;
  std::cout << "Updated P" << std::endl << p_ << std::endl;
  return Estimate(x_, nis_);
}

// Private Members
// -----------------------------------------------------------------------------

// TODO: move repeating sigma generation code here
//void UkfTracker::GenerateSigmaPoints()

void UkfTracker::Predict(float dt, Eigen::MatrixXf& x_sig_pred) {
  // Create sigma point matrix
  Eigen::MatrixXf x_sig(kNx, kNsigX);
  // Set the first sigma point to the state mean
  x_sig.col(0) = x_;
  // Calculate the square root clause of lambda + nx multiplied by P
  Eigen::MatrixXf p_sq = p_.llt().matrixL();
  Eigen::MatrixXf sq = std::sqrt(kLambdaX + kNx) * p_sq;
  // Set the rest of sigma points
  x_sig.middleCols(1, kNx) = sq.colwise() + x_;
  x_sig.middleCols(kNx + 1, kNx) = (-sq).colwise() + x_;

  // Create augmented mean vector
  Eigen::VectorXf x_aug(Eigen::VectorXf::Zero(kNaug));
  x_aug.head(kNx) = x_;
  // Create augmented state covariance
  Eigen::MatrixXf p_aug(Eigen::MatrixXf::Zero(kNaug, kNaug));
  p_aug.topLeftCorner(kNx, kNx) = p_;
  p_aug.bottomRightCorner(kNq, kNq) = kQ;
//  std::cout << "p_aug" << std::endl << p_aug << std::endl;

  // Create augmented sigma point matrix
  Eigen::MatrixXf x_sig_aug(kNaug, kNsigAug);
  // Set the first sigma point to the augmented state mean
  x_sig_aug.col(0) = x_aug;
  // Calculate the square root clause of lambda + nx multiplied by P
  Eigen::MatrixXf p_aug_sq = p_aug.llt().matrixL();
  Eigen::MatrixXf sq_aug = std::sqrt(kLambdaAug + kNaug) * p_aug_sq;
  // Set the rest of sigma points
  x_sig_aug.middleCols(1, kNaug) = sq_aug.colwise() + x_aug;
  x_sig_aug.middleCols(kNaug + 1, kNaug) = (-sq_aug).colwise() + x_aug;
//  std::cout << "x_sig_aug" << std::endl << x_sig_aug << std::endl;

  // Create matrix with predicted sigma points as columns
  for (auto i = 0; i < kNsigAug; ++i) {
    Eigen::VectorXf noise(kNx);
    // TODO: optimize x_sig_aug.col(i)
    noise(0) = dt * dt / 2 * std::cos(x_sig_aug.col(i)(3)) * x_sig_aug.col(i)(5);
    noise(1) = dt * dt / 2 * std::sin(x_sig_aug.col(i)(3)) * x_sig_aug.col(i)(5);
    noise(2) = dt * x_sig_aug.col(i)(5);
    noise(3) = dt * dt / 2 * x_sig_aug.col(i)(6);
    noise(4) = dt * x_sig_aug.col(i)(6);
    
    Eigen::VectorXf x1(Eigen::VectorXf::Zero(kNx));
    if (std::fabs(x_sig_aug.col(i)(4)) < kEpsilon) {
      x1(0) = x_sig_aug.col(i)(2) * std::cos(x_sig_aug.col(i)(3)) * dt;
      x1(1) = x_sig_aug.col(i)(2) * std::sin(x_sig_aug.col(i)(3)) * dt;
    } else {
      x1(0) = x_sig_aug.col(i)(2) / x_sig_aug.col(i)(4)
        * (std::sin(x_sig_aug.col(i)(3) + x_sig_aug.col(i)(4) * dt)
         - std::sin(x_sig_aug.col(i)(3)));
      x1(1) = x_sig_aug.col(i)(2) / x_sig_aug.col(i)(4)
        * (-std::cos(x_sig_aug.col(i)(3) + x_sig_aug.col(i)(4) * dt)
          + std::cos(x_sig_aug.col(i)(3)));
      x1(3) = x_sig_aug.col(i)(4) * dt;
    }
    x_sig_pred.col(i) = x_sig_aug.col(i).head(kNx) + x1 + noise;
  }
//  std::cout << "x_sig_pred" << std::endl << x_sig_pred << std::endl;

  // Predict state mean
  x_ = x_sig_pred * kAugWeights;
//  std::cout << "x_" << std::endl << x_ << std::endl;
  // Predict state covariance matrix
  Eigen::MatrixXf diff(kNx, kNsigAug);
  diff = x_sig_pred.colwise() - x_;
//  std::cout << "diff" << std::endl << diff << std::endl;
  // Angle normalization
//  std::cout << "Normalizing angle..." << std::endl;
  for (auto i = 0; i < kNsigAug; ++i) {
//    std::cout << "diff(3, " << i << ") " << diff(3, i) << std::endl;
    while (diff(3, i) > M_PI) {
      diff(3, i) -= 2. * M_PI;
    }
    while (diff(3, i) < -M_PI) {
      diff(3, i) += 2. * M_PI;
    }
  }
  p_ = diff * kAugWeights.asDiagonal() * diff.transpose();
}

void UkfTracker::Update(const Eigen::VectorXf& z,
                        const Eigen::MatrixXf& x_sig_pred,
                        const Eigen::MatrixXf& z_sig,
                        const Eigen::VectorXf& z_pred,
                        const Eigen::MatrixXf& s) {
  // Differences of state sigma points
  Eigen::MatrixXf x_diffs = x_sig_pred.colwise() - x_;
  // Angle normalization
  for (auto i = 0; i < kNsigAug; ++i) {
    while (x_diffs(3, i) > M_PI) {
      x_diffs(3, i) -= 2. * M_PI;
    }
    while (x_diffs(3, i) < -M_PI) {
      x_diffs(3, i) += 2. * M_PI;
    }
  }
  // Differences of measurement sigma points
  Eigen::MatrixXf z_diffs = z_sig.colwise() - z_pred;
  // Angle normalization
  for (auto i = 0; i < kNsigAug; ++i) {
    while (z_diffs(1, i) > M_PI) {
      z_diffs(1, i) -= 2. * M_PI;
    }
    while (z_diffs(1, i) < -M_PI) {
      z_diffs(1, i) += 2. * M_PI;
    }
  }
  // Create matrix for cross correlation Tc
  Eigen::MatrixXf tc = x_diffs * kAugWeights.asDiagonal() * z_diffs.transpose();
  // Calculate Kalman gain K;
  Eigen::MatrixXf k = tc * s.inverse();
//  std::cout << "k" << std::endl << k << std::endl;
  // Measurement difference
  Eigen::VectorXf z_diff = z - z_pred;
  nis_ = z_diff.transpose() * s.inverse() * z_diff;
//  std::cout << "NIS" << std::endl << nis_ << std::endl;
  // Angle normalization
  while (z_diff(1) > M_PI) {
    z_diff(1) -= 2. * M_PI;
  }
  while (z_diff(1) < -M_PI) {
    z_diff(1) += 2. * M_PI;
  }
//  std::cout << "z_diff" << std::endl << z_diff << std::endl;
  // Update state mean and covariance matrix
  x_ += k * z_diff;
  p_ -= k * s * k.transpose();
}

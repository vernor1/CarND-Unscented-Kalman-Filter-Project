#include "ukf_tracker.h"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// State dimension
const unsigned short kNx = 5;

// Number of sigma points
const unsigned short kNsigX = 2 * kNx + 1;

// Augmented state dimension
const unsigned short kNaug = 7;

// Number of augmented sigma points
const unsigned short kNsigAug = 2 * kNaug + 1;

// 2*Pi, needed for angle normalization
const float k2pi = 2. * M_PI;

// The square root of 3, needed for generating sigma points
const float kSqrt3 = std::sqrt(3);

// Process noise standard deviation longitudinal acceleration in m/s^2.
// Assuming that acceleration on first 10 m is 2 m/s^2, standard
// deviation is about half of that, resulting in 1.0:
// http://www.analyticcycling.com/DiffEqMotionFunctions_Page.html
const float kStdA = 1.0;

// Process noise standard deviation yaw acceleration in rad/s^2.
// Pi/5 seems to be a reasonable value, showing a good RMSE.
const float kStdYaw = M_PI / 5;

// Augmented covariance dimension
const unsigned short kNq = 2;

// Augmented covariance matrix
const Eigen::MatrixXf kQ((Eigen::MatrixXf(kNq, kNq)
  << kStdA * kStdA, 0,
     0, kStdYaw * kStdYaw).finished());

// A value below the absolute constant is considered zero
const float kEpsilon = 1e-6;

// Augmented weights
Eigen::VectorXf CreateWeights(unsigned short n_points) {
  auto double_n_points = 2 * n_points;
  Eigen::VectorXf weights(double_n_points + 1);
  float base_weight = 1. / 3;
  weights(0) = base_weight * (3 - n_points);
  weights.tail(double_n_points).setConstant(base_weight / 2);
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

// Initial state covariance matrix
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

void NormalizeAngle(float& a) {
  if (a > M_PI || a < -M_PI) {
    // a=(a+Pi)-2*Pi*floor((a+Pi)/(2*Pi))
    a -= M_PI * (1 + 2 * std::floor(a / M_PI));
    // Adjust angle if needed
    if (a >= k2pi) {
      a = 0;
    } else if (a < 0) {
      a += k2pi;
    }
    a -= M_PI;
  }
}

Eigen::MatrixXf GenerateSigmaPoints(size_t n_sig,
                                    const Eigen::VectorXf& x,
                                    const Eigen::MatrixXf& p) {
  auto n_x = x.size();
  // Create sigma point matrix
  Eigen::MatrixXf x_sig(n_x, n_sig);
  // Set the first sigma point to the state mean
  x_sig.col(0) = x;
  // Calculate the square root clause of lambda + nx multiplied by P
  Eigen::MatrixXf p_sq = p.llt().matrixL();
  Eigen::MatrixXf sq = kSqrt3 * p_sq;
  // Set the rest of sigma points
  x_sig.middleCols(1, n_x) = sq.colwise() + x;
  x_sig.middleCols(n_x + 1, n_x) = (-sq).colwise() + x;
  return x_sig;
}

void PredictLidarMeasurement(const Eigen::MatrixXf& x_sig_pred,
                             Eigen::MatrixXf& z_sig,
                             Eigen::VectorXf& z_pred,
                             Eigen::MatrixXf& s) {
  // Create matrix for sigma points in measurement space
  z_sig = Eigen::MatrixXf(kNzLidar, kNsigAug);
  // Transform sigma points into measurement space
  for (auto i = 0; i < kNsigAug; ++i) {
    z_sig.col(i) = x_sig_pred.col(i).head(kNzLidar);
  }
  // Calculate mean predicted measurement
  z_pred = z_sig * kAugWeights;
  // Calculate measurement covariance matrix S
  Eigen::MatrixXf diff = z_sig.colwise() - z_pred;
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
    auto px = x_sig_pred(0, i);
    auto py = x_sig_pred(1, i);
    auto v = x_sig_pred(2, i);
    auto psi = x_sig_pred(3, i);
    auto sqrt_of_sum_of_squares = std::sqrt(px * px + py * py);
    z_sig(0, i) = sqrt_of_sum_of_squares;
    z_sig(1, i) = std::fabs(px) < kEpsilon ? 0 : std::atan2(py, px);
    z_sig(2, i) = std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon ?
      0 :
      v * (px * std::cos(psi) + py * std::sin(psi)) / sqrt_of_sum_of_squares;
  }
  // Calculate mean predicted measurement
  z_pred = z_sig * kAugWeights;
  // Calculate measurement covariance matrix S
  Eigen::MatrixXf diff = z_sig.colwise() - z_pred;
  s = diff * kAugWeights.asDiagonal() * diff.transpose() + kRradar;
}

} // namespace

// Public Members
// -----------------------------------------------------------------------------

UkfTracker::UkfTracker()
  : is_initialized_(false),
    previous_timestamp_(-1),
    x_(Eigen::VectorXf::Zero(kNx)),
    p_(kPinitial) { }

UkfTracker::Estimate UkfTracker::operator()(const Measurement& measurement) {
  if (!is_initialized_) {
    // Initialize the state with the first measurement
    if (measurement.sensor_type == Measurement::SensorType::kRadar) {
      x_ = ConvertPolarToCartesian(measurement.value);
    }
    else {
      x_ << measurement.value(0), measurement.value(1), 0, 0, 0;
    }
    // The ground truth data indicates the bicycle speed is almost constant
    // throughout the experiemtent, and it's about 5 m/s
    x_(2) = 5;
    // We are quite sure about the initial px and py
    p_(0, 0) = 0.1;
    p_(1, 1) = 0.1;
    // Initialize the previous timestamp with the first measurement
    previous_timestamp_ = measurement.timestamp;
    is_initialized_ = true;

    return Estimate(x_, 0);
  }

  // Time elapsed between the current and previous measurements in seconds
  float dt = (measurement.timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement.timestamp;

  // Predict state mean and covariance
  Eigen::MatrixXf x_sig_pred(kNx, kNsigAug);
  Predict(dt, x_sig_pred);

  // Predict measurement mean and covariance
  Eigen::MatrixXf z_sig;
  Eigen::VectorXf z_pred;
  Eigen::MatrixXf s;
  if (measurement.sensor_type == Measurement::SensorType::kRadar) {
    PredictRadarMeasurement(x_sig_pred, z_sig, z_pred, s);
  } else {
    PredictLidarMeasurement(x_sig_pred, z_sig, z_pred, s);
  }

  // Update state mean and covariance
  auto nis = Update(x_sig_pred, measurement.value, z_sig, z_pred, s);

  return Estimate(x_, nis);
}

// Private Members
// -----------------------------------------------------------------------------

void UkfTracker::Predict(float dt, Eigen::MatrixXf& x_sig_pred) {
  // Create sigma point matrix
  Eigen::MatrixXf x_sig = GenerateSigmaPoints(kNsigX, x_, p_);
  // Create augmented mean vector
  Eigen::VectorXf x_aug(Eigen::VectorXf::Zero(kNaug));
  x_aug.head(kNx) = x_;
  // Create augmented state covariance
  Eigen::MatrixXf p_aug(Eigen::MatrixXf::Zero(kNaug, kNaug));
  p_aug.topLeftCorner(kNx, kNx) = p_;
  p_aug.bottomRightCorner(kNq, kNq) = kQ;
  // Create augmented sigma point matrix
  Eigen::MatrixXf x_sig_aug = GenerateSigmaPoints(kNsigAug, x_aug, p_aug);

  // Create matrix with predicted sigma points as columns
  for (auto i = 0; i < kNsigAug; ++i) {
    auto v = x_sig_aug(2, i);
    auto psi = x_sig_aug(3, i);
    auto psi_dot = x_sig_aug(4, i);
    auto nu_a = x_sig_aug(5, i);
    auto nu_psi_dd = x_sig_aug(6, i);
    auto dt_by_2 = dt / 2;
    auto dt_nu_psi_dd = dt * nu_psi_dd;
    auto dt_nu_a = dt * nu_a;
    auto dt2_by_2_nu_a = dt_nu_a * dt_by_2;
    Eigen::VectorXf noise(kNx);
    noise(0) = dt2_by_2_nu_a * std::cos(psi);
    noise(1) = dt2_by_2_nu_a * std::sin(psi);
    noise(2) = dt_nu_a;
    noise(3) = dt_nu_psi_dd * dt_by_2;
    noise(4) = dt_nu_psi_dd;
   
    auto v_dt = v * dt;
    auto dt_psi_dot = dt * psi_dot;
    auto dt_psi_dot_plus_psi = dt_psi_dot + psi;
    auto v_by_psi_dot = v / psi_dot;
    Eigen::VectorXf x1(Eigen::VectorXf::Zero(kNx));
    if (std::fabs(psi_dot) < kEpsilon) {
      x1(0) = v_dt * std::cos(psi);
      x1(1) = v_dt * std::sin(psi);
    } else {
      x1(0) = v_by_psi_dot * (std::sin(dt_psi_dot_plus_psi) - std::sin(psi));
      x1(1) = v_by_psi_dot * (-std::cos(dt_psi_dot_plus_psi) + std::cos(psi));
      x1(3) = dt_psi_dot;
    }
    x_sig_pred.col(i) = x_sig_aug.col(i).head(kNx) + x1 + noise;
  }

  // Predict state mean
  x_ = x_sig_pred * kAugWeights;
  // Predict state covariance matrix
  Eigen::MatrixXf diff(kNx, kNsigAug);
  diff = x_sig_pred.colwise() - x_;
  for (auto i = 0; i < kNsigAug; ++i) {
    NormalizeAngle(diff(3, i));
  }
  // Even though turning the weights vector into a diagonal matrix before the
  // multiplication might seem inefficient, but this vectorized multiplication
  // below shows a significantly better performance, than an explicit loop over
  // sigma points, for instance:
  //  Eigen::MatrixXf product(m1.rows(), m2.rows());
  //  product.fill(0);
  //   for (auto i = 0; i < m1.cols(); ++i) {
  //     product += v(i) * m1.col(i) * m2.col(i).transpose();
  //   }
  // When all occurences of such multiplication are replaced with the loop,
  // the completion  time over 8000 samples is about 3s (on an Intel i7). The
  // current vectorized completion time is 2.1s. It must be a result of the
  // optimal Eigen matrix multiplication algorithm.
  p_ = diff * kAugWeights.asDiagonal() * diff.transpose();
}

float UkfTracker::Update(const Eigen::MatrixXf& x_sig_pred,
                         const Eigen::VectorXf& z,
                         const Eigen::MatrixXf& z_sig,
                         const Eigen::VectorXf& z_pred,
                         const Eigen::MatrixXf& s) {
  // Differences of state sigma points
  Eigen::MatrixXf x_diffs = x_sig_pred.colwise() - x_;
  for (auto i = 0; i < kNsigAug; ++i) {
    NormalizeAngle(x_diffs(3, i));
  }
  // Differences of measurement sigma points
  Eigen::MatrixXf z_diffs = z_sig.colwise() - z_pred;
  for (auto i = 0; i < kNsigAug; ++i) {
    NormalizeAngle(z_diffs(1, i));
  }
  // Create matrix for cross correlation Tc
  Eigen::MatrixXf tc = x_diffs * kAugWeights.asDiagonal() * z_diffs.transpose();
  // Calculate Kalman gain K;
  Eigen::MatrixXf k = tc * s.inverse();
  // Measurement difference
  Eigen::VectorXf z_diff = z - z_pred;
  NormalizeAngle(z_diff(1));
  // Update state mean and covariance matrix
  x_ += k * z_diff;
  p_ -= k * s * k.transpose();
  // Return NIS
  return z_diff.transpose() * s.inverse() * z_diff;
}

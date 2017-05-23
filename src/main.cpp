#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "ukf_tracker.h"

// Local Types
// -----------------------------------------------------------------------------

typedef std::vector<UkfTracker::Measurement> MeasurementSequence;
typedef std::vector<UkfTracker::Estimate> EstimateSequence;
typedef std::vector<Eigen::VectorXf> GroundTruthSequence;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks arguments of the program and exits if the check fails.
// @param[in] argc  Number of arguments
// @param[in] argv  Array of arguments
void CheckArguments(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // Make sure the user has provided input and output files
  if (argc == 1) {
    std::cerr << usage_instructions << std::endl;
  } else if (argc == 2) {
    std::cerr << "Please include an output file." << std::endl
              << usage_instructions << std::endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    std::cerr << "Too many arguments." << std::endl << usage_instructions
              << std::endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

// Checks input and output files and exits if the check fails.
// @param[in] in_file        Input file
// @param[in] in_file_name   Input file name
// @param[in] out_file       Output file
// @param[in] out_file_name  Output file name
void CheckFiles(const std::ifstream& in_file,
                const std::string& in_file_name,
                const std::ofstream& out_file,
                const std::string& out_file_name) {
  if (!in_file.is_open()) {
    std::cerr << "Cannot open input file: " << in_file_name << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    std::cerr << "Cannot open output file: " << out_file_name << std::endl;
    exit(EXIT_FAILURE);
  }
}

// Loads input data.
// @param[in]  in_file                Input file
// @param[out] measurement_sequence   Container for measurements data
// @param[out] ground_truth_sequence  Container for ground truth data
void LoadData(std::ifstream& in_file,
              MeasurementSequence& measurement_sequence,
              GroundTruthSequence& ground_truth_sequence) {
  std::string line;
  while (std::getline(in_file, line)) {
    // Each line represents a measurement at a timestamp
    std::istringstream iss(line);
    long long timestamp;
    std::string sensor_type;
    UkfTracker::Measurement measurement;

    // Read the first element from the current line
    iss >> sensor_type;
    if (sensor_type == "L") {
      // Laser measurement
      measurement.sensor_type = UkfTracker::Measurement::SensorType::kLidar;
      measurement.value = Eigen::VectorXf(2);
      float x;
      float y;
      iss >> x >> y >> timestamp;
      measurement.value << x, y;
      measurement.timestamp = timestamp;
      measurement_sequence.push_back(measurement);
    } else if (sensor_type == "R") {
      // Radar measurement
      measurement.sensor_type = UkfTracker::Measurement::SensorType::kRadar;
      measurement.value = Eigen::VectorXf(3);
      float rho;
      float phi;
      float rho_dot;
      iss >> rho >> phi >> rho_dot >> timestamp;
      measurement.value << rho, phi, rho_dot;
      measurement.timestamp = timestamp;
      measurement_sequence.push_back(measurement);
    } else {
      continue;
    }

    // Read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    Eigen::VectorXf groundTruth(4);
    groundTruth << x_gt, y_gt, vx_gt, vy_gt;
    ground_truth_sequence.push_back(groundTruth);
  }
}

// Dumps all estimates, measurements and ground truth data to a file.
// @param[in]  measurement_sequence   Container with measurements data
// @param[in]  ground_truth_sequence  Container with ground truth data
// @param[in]  estimate_sequence      Container with esimations data
// @param[out] out_file               Output file
void DumpData(const EstimateSequence& estimate_sequence,
              const MeasurementSequence& measurement_sequence,
              const GroundTruthSequence& ground_truth_sequence,
              std::ofstream& out_file) {
  assert(estimate_sequence.size() == measurement_sequence.size()
      && measurement_sequence.size() == ground_truth_sequence.size());

  // Column names for output file
  out_file << "time_stamp" << "\t";  
  out_file << "px_state" << "\t";
  out_file << "py_state" << "\t";
  out_file << "v_state" << "\t";
  out_file << "yaw_angle_state" << "\t";
  out_file << "yaw_rate_state" << "\t";
  out_file << "sensor_type" << "\t";
  out_file << "NIS" << "\t";  
  out_file << "px_measured" << "\t";
  out_file << "py_measured" << "\t";
  out_file << "px_ground_truth" << "\t";
  out_file << "py_ground_truth" << "\t";
  out_file << "vx_ground_truth" << "\t";
  out_file << "vy_ground_truth" << "\n";

  for (auto i = 0; i < measurement_sequence.size(); ++i) {
    auto estimate = estimate_sequence[i];
    auto measurement = measurement_sequence[i];
    auto groundTruth = ground_truth_sequence[i];

    // Output the timestamp
    out_file << measurement.timestamp << "\t";

    // Output the estimated state
    out_file << estimate.x(0) << "\t" << estimate.x(1) << "\t"
             << estimate.x(2) << "\t" << estimate.x(3) << "\t"
             << estimate.x(4) << "\t";

    if (measurement.sensor_type
        == UkfTracker::Measurement::SensorType::kLidar) {
      // Output the sensor type
      out_file << "lidar" << "\t";
      // Output the NIS value
      out_file << estimate.nis << "\t";
      // Output the  measurement px and py
      out_file << measurement.value(0) << "\t" << measurement.value(1) << "\t";
    } else {
      // Output the sensor type
      out_file << "radar" << "\t";
      // Output the NIS value
      out_file << estimate.nis << "\t";
      // Output the  measurement px and py
      auto rho = measurement.value(0);
      auto phi = measurement.value(1);
      out_file << rho * cos(phi) << "\t" << rho * sin(phi) << "\t";
    }

    // Output the ground truth
    out_file << groundTruth(0) << "\t" << groundTruth(1) << "\t"
             << groundTruth(2) << "\t" << groundTruth(3) << "\n";
  }
}

// Calculates RMSE of the estimates and ground truth data.
// @param[in] estimate_sequence      Container with esimations data
// @param[in] ground_truth_sequence  Container with ground truth data
// @return The Eigen vector containing the RMSE values for px, py, vs, vy
Eigen::VectorXf CalculateRmse(
  const EstimateSequence& estimate_sequence,
  const GroundTruthSequence& ground_truth_sequence) {

  Eigen::VectorXf rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimate vector size should equal ground truth vector size
  if (estimate_sequence.size() != ground_truth_sequence.size()
      || estimate_sequence.empty()) {
    std::cout << "Invalid input!" << std::endl;
    return rmse;
  }
  // Accumulate squared differences
  for (auto i = 0; i < estimate_sequence.size(); ++i) {
    Eigen::VectorXf ukf_x = estimate_sequence[i].x;
    // Convert UKF x vector to cartesian to compare to ground truth
    Eigen::VectorXf ukf_x_cartesian(4);

    auto x_estimate = ukf_x(0);
    auto y_estimate = ukf_x(1);
    auto vx_estimate = ukf_x(2) * std::cos(ukf_x(3));
    auto vy_estimate = ukf_x(2) * std::sin(ukf_x(3));
    
    ukf_x_cartesian << x_estimate, y_estimate, vx_estimate, vy_estimate;
    rmse = rmse.array()
      + (ukf_x_cartesian - ground_truth_sequence[i]).array().pow(2);
  }
  rmse = (rmse / estimate_sequence.size()).array().sqrt();

  return rmse;
}

void GetKthHigestNis(
  const MeasurementSequence& measurement_sequence,
  const EstimateSequence& estimate_sequence,
  size_t k,
  float& lidarNis,
  float& radarNis) {

  assert(measurement_sequence.size() == estimate_sequence.size());
  assert(k < estimate_sequence.size());
  std::vector<float> lidarNisSequence;
  std::vector<float> radarNisSequence;
  for (auto i = 0; i < measurement_sequence.size(); ++i) {
    if (measurement_sequence[i].sensor_type
        == UkfTracker::Measurement::SensorType::kLidar) {
      lidarNisSequence.push_back(estimate_sequence[i].nis);
    } else {
      radarNisSequence.push_back(estimate_sequence[i].nis);
    }
  }
  std::sort(lidarNisSequence.rbegin(), lidarNisSequence.rend());
  std::sort(radarNisSequence.rbegin(), radarNisSequence.rend());
  lidarNis = lidarNisSequence[k];
  radarNis = radarNisSequence[k];
}

// main
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  CheckArguments(argc, argv);

  // Open and check files
  std::ifstream in_file(argv[1], std::ifstream::in);
  std::ofstream out_file(argv[2], std::ofstream::out);

  CheckFiles(in_file, argv[1], out_file, argv[2]);

  // Containers for loading measurements and ground truth data
  std::vector<UkfTracker::Measurement> measurement_sequence;
  std::vector<Eigen::VectorXf> ground_truth_sequence;

  // Load input data and close the file
  LoadData(in_file, measurement_sequence, ground_truth_sequence);
  if (in_file.is_open()) {
    in_file.close();
  }

  // Generate estimate data
  UkfTracker ukf_tracker;
  EstimateSequence estimate_sequence;
  std::transform(measurement_sequence.begin(), measurement_sequence.end(),
                 std::back_inserter(estimate_sequence),
                 ukf_tracker);

  // Dump all data and close the file
  DumpData(estimate_sequence, measurement_sequence, ground_truth_sequence,
           out_file);
  if (out_file.is_open()) {
    out_file.close();
  }

  // NIS estimation
  float lidarNis = 0;
  float radarNis = 0;
  GetKthHigestNis(measurement_sequence,
                  estimate_sequence,
                  estimate_sequence.size() * 0.05,
                  lidarNis,
                  radarNis);
  std::cout << "5\% of NIS estimates are higher than "
            << lidarNis << " (lidar), " << radarNis << " (radar)" << std::endl;

  // Compute the accuracy (RMSE)
  std::cout << "RMSE" << std::endl;
  std::cout << CalculateRmse(estimate_sequence, ground_truth_sequence)
            << std::endl;

  return 0;
}

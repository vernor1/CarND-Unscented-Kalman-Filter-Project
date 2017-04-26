# Extended Kalman Filter Project

The goals / steps of this project are the following:

* Complete the Extended Kalman Filter algorithm in C++.
* Ensure that your project compiles.
* Test your Kalman Filter against the sample data. Ensure that the px, py, vx, and vy RMSE are below the values specified in the rubric.

## [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Compiling
#### 1. Your code should compile.  

The code compiles without errors with cmake-3.7.2 and make-3.81 on macOS-10.12.4.

---
### Accuracy
#### 1. The px, py, vx, vy output coordinates have an RMSE <= [0.08, 0.08, 0.60, 0.60] when using the file: "sample-laser-radar-measurement-data-1.txt".

The computationally stable RMSE on `sample-laser-radar-measurement-data-1.txt` is `[0.0651653, 0.0605378, 0.543192, 0.544191]`.


#### 2. The px, py, vx, vy output coordinates have an RMSE <= [0.20, 0.20, .50, .85] when using the file: "sample-laser-radar-measurement-data-2.txt".

The computationally stable RMSE on `sample-laser-radar-measurement-data-2.txt` is `[0.185497, 0.190302, 0.476754, 0.804467]`.

---
### Follows the Correct Algorithm
#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The implemented solution follows the Sensor Fusion method, including the Extended Kalman Filter, which is described in the lesson.


#### 2. Your Kalman Filter algorithm handles the first measurements appropriately.

The first measurement is handled by `EkfTracker::operator()` defined in `ekf_tracker.cpp`.


#### 3. Your Kalman Filter algorithm first predicts then updates.

The method `EkfTracker::Predict` is called before `EkfTracker::LidarUpdate` or `EkfTracker::RadarUpdate` in `EkfTracker::operator()`.


#### 4. Your Kalman Filter can handle radar and lidar measurements.

Method `EkfTracker::LidarUpdate` uses the constant measurement matrix `kH`, which is used along with the constant measurement covariance matrix `kRlidar` for estimating new state `x_` and state covariance matrix `P_`.

Method `EkfTracker::RadarUpdate` uses the function `ConvertCartesianToPolar` to compute `h(x')` and `CalculateJacobian` to compute the Jacobian matrix `Hj`, which are used along with the constant measurement covariance matrix `kRradar` for estimating new state `x_` and state covariance matrix `P_`.

---
### Code Efficiency
#### 1. Your algorithm should avoid unnecessary calculations.

Most duplicating expensive computations (like floating point multiplications and divisions) are done only once and then reused, see functions `CalculateJacobian`, `ConvertCartesianToPolar`, `EkfTracker::Predict`, `EkfTracker::LidarUpdate`, `EkfTracker::RadarUpdate` in `ekf_tracker.cpp`. All constant matrices are moved to a nameless namespace of `ekf_tracker.cpp`.

---
### Notes

* The code is complying with the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
* The main Sensor Fusion class EkfTracker is made a functor, which is easy to plug into an algorithm generating estimations out from measurements, see the use of `std::transform` in function `main` of `main.cpp`
* The C++11 automatic type deduction is used wherever appropriate.
* All function/method comments are made Doxygen-friendly

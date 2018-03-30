#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  laser_flag_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  radar_flag_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Initial time is zero
  time_us_ = 0;

  // state vector dimensions
  n_x_ = 5;

  // augmented state vector dimensions
  n_aug_ = 7;

  // initialise lambda parameter for calculating sigma points
  lambda_ = 3 - n_aug_;

  // initialise weights based on lambda and
  weights_ = VectorXd(2*n_aug_ + 1);

  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  Xsig_pred_ =  MatrixXd(n_x_, 2*n_aug_ + 1);

  e_NIS_radar = 0;
  e_NIS_lidar = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO: done
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  //
  if (!is_initialized_) {
    // Let user know what's going on
    std::cout << "UKF. Initialisation..." << endl;

    // Initialise state x_
    x_ << 1, 1, 1, 1, 1;

    // Initialise covariance matrix P_
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Check if a new measurement came from radar or lidar
    if (meas_package.sensor_type_ == meas_package.RADAR && radar_flag_) {
      // process radar measurements
      double rho = meas_package.raw_measurements_[0];
      double psi = meas_package.raw_measurements_[1];

      x_ << rho*cos(psi), rho*sin(psi), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == meas_package.LASER && laser_flag_) {
      // process lidar measurements
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

    }
    else {
      // Complain if a new measurement neither lidar or radar
      std::cout << "The  Measurement has been skipped." << endl;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  // Calculate delta t in seconds:
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  if (meas_package.sensor_type_ == meas_package.RADAR && radar_flag_)
    UpdateRadar(meas_package.raw_measurements_);
  else if (meas_package.sensor_type_ == meas_package.LASER && laser_flag_)
    UpdateLidar(meas_package.raw_measurements_);
  else
    std::cout << "The  Measurement has been skipped." << endl;

  // print the output
  //cout << "x_ = " << endl << x_ << endl;
  //cout << "P_ = " << endl << P_ << endl;
  //cout << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO: done

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  Sigma_Point_Prediction(Augmented_Sigma_Points(), delta_t);
  Predict_Mean_Covariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(VectorXd measurement) {
  /**
  TODO: done

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  const int n_z = 2;
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S_pred = MatrixXd(n_z,n_z);

  Predict_Lidar_Measurement(&z_pred, &S_pred);
  Update_Lidar_State(measurement, z_pred, S_pred);

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(VectorXd measurement) {
  /**
  TODO: done

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  const int n_z = 3;
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S_pred = MatrixXd(n_z,n_z);

  Predict_Radar_Measurement(&z_pred, &S_pred);
  Update_Radar_State(measurement, z_pred, S_pred);
}

/*
 *  Prediction stage methods
 *  --------------------------------------------------------------------------------
 */



// This method calculates Xsig_pred_
void UKF::Sigma_Point_Prediction(MatrixXd Xsig_aug_, double delta_t) {

  //predict sigma points
  for (int i = 0; i < 2*n_aug_ + 1; i++) {

    // extract components of each column
    double v = Xsig_aug_(2, i);
    double psi = Xsig_aug_(3, i);
    double psi_d = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_psi_dd = Xsig_aug_(6, i);


    VectorXd x_old = Xsig_aug_.col(i).head(n_x_);
    VectorXd x_1 = VectorXd(5);
    VectorXd x_2 = VectorXd(5);

    double eps = 0.01;
    //avoid division by zero
    if (fabs(psi_d) < eps)
      x_1 <<  v * cos(psi) * delta_t,
              v * sin(psi) * delta_t,
              0,
              0,
              0;
    else
      x_1 <<  v / psi_d * (sin(psi + psi_d * delta_t) - sin(psi)),
              v / psi_d * (-cos(psi + psi_d * delta_t) + cos(psi)),
              0,
              psi_d * delta_t,
              0;

    x_2 <<    0.5 * pow(delta_t, 2) * cos(psi) * nu_a,
              0.5 * pow(delta_t, 2) * sin(psi) * nu_a,
              delta_t * nu_a,
              0.5 * pow(delta_t, 2) * nu_psi_dd,
              delta_t * nu_psi_dd;

    //write predicted spigma points into right column
    Xsig_pred_.col(i) = x_old + x_1 + x_2;
  }
}


// Given P_, x_, std_a_, std_yawdd_, lambda this method calculates
// matrix Xsig_aug_ with columns corresponding to augmented sigma points
MatrixXd UKF::Augmented_Sigma_Points() {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // set all elements of vector x_aug to zero
  x_aug.setZero();
  // set first n_x_ elements of x_aug to x_
  x_aug.head(n_x_) = x_;

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  // set P_aug entries to zero
  P_aug.setZero();
  // matrix P_aug consists of P_ and Q on its diagonal
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // predict sigma points
  // create matrix with sigma points in its columns
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_ + 1);
  Xsig_aug_.setZero();
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i);
    Xsig_aug_.col(i+n_aug_+1) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i);
  }
  return Xsig_aug_;
}




// Based on Xsig_pred_ this method predicts x_ and P_
void UKF::Predict_Mean_Covariance() {

  x_.setZero();
  P_.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = Normalization_of_Angle(x_diff(3));
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

/*
 * --------------------------------------------------------------------------------
 * Update stage methods
 * --------------------------------------------------------------------------------
 */


// This method propgates sigma points through measurement function for radar, as the radar measurement cannot be directly
void UKF::Predict_Radar_Measurement(VectorXd* z_out, MatrixXd* S_out) {

  const int n_z = 3;

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  z_pred.setZero();
  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double px =       Xsig_pred_(0, i);
    double py =       Xsig_pred_(1, i);
    double v =        Xsig_pred_(2, i);
    double psi =      Xsig_pred_(3, i);
    double psi_d =    Xsig_pred_(4, i);

    // introduce small eps in order to avoid division by zero
    double eps =      0.01;

    double rho =      sqrt(pow(px, 2) + pow(py, 2));
    double phi =      atan2(py, px);
    double rho_d =    (px*v*cos(psi) + py*v*sin(psi)) / std::max(rho, eps);

    Zsig_.col(i) <<   rho,
                      phi,
                      rho_d;

    // calculate mean predicted measurement
    z_pred += weights_(i) * Zsig_.col(i);
  }

  // calculate innovation covariance matrix S
  // create measurement noise matrix R first
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  // set entries of S to zero
  S.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    z_diff(1) = Normalization_of_Angle(z_diff(1));
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R;

  *S_out = S;
  *z_out = z_pred;
}

//THis method will normalize the angle by adding or subtracting 2*pi.
double UKF::Normalization_of_Angle(double& angle) {
  if (angle > M_PI) {
    angle -= 2.0*M_PI;
  }
  else if (angle < -M_PI) {
    angle += 2.0*M_PI;
  }
  return angle;
}





// This method propagates sigma points through measurement function for Lidar
void UKF::Predict_Lidar_Measurement(VectorXd* z_out, MatrixXd* S_out) {

  const int n_z = 2;

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  z_pred.setZero();
  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double px =     Xsig_pred_(0, i);
    double py =     Xsig_pred_(1, i);

    Zsig_.col(i) << px,
                    py;

    //calculate mean predicted measurement
    z_pred +=       weights_(i) * Zsig_.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  S.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R;

  *S_out = S;
  *z_out = z_pred;
}

// This method updates state and covariance using
// predicted radar measurement vector, current measurement and
// predicted covariance matrix.
void UKF::Update_Radar_State(VectorXd& z, VectorXd& z_pred, MatrixXd& S) {

  const int n_z = 3;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    x_diff(3) = Normalization_of_Angle(x_diff(3));
    z_diff(1) = Normalization_of_Angle(z_diff(1));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z, n_z);

  double eps = 0.01;
  if (S.norm() < eps) {
    K = Tc * (S + eps*MatrixXd::Identity(n_z, n_z)).inverse();
  } else {
    K = Tc * S.inverse();
  }


  VectorXd z_diff = z - z_pred;
  z_diff(1) = Normalization_of_Angle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate NIS
  e_NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
}

// This method updates state (x_) and covariance (P_) using
// predicted lidar measurement vector, current measurement and
// predicted covariance matrix
void UKF::Update_Lidar_State(VectorXd& z, VectorXd& z_pred, MatrixXd& S) {

  const int n_z = 2;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig_.col(i) - z_pred;
    x_diff(3) = Normalization_of_Angle(x_diff(3));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z, n_z);
  K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate NIS
  e_NIS_lidar = z_diff.transpose() * S.inverse() * z_diff;
}

#include "kalman_filter.h"
#include "../Eigen/Dense" // using Eigen 5

// simulate a robot moving velocity of sqrt(2) with vx = vy = 1
using namespace Eigen;

KalmanFilter::KalmanFilter(Vector4d state_estimate, Matrix4d state_covar,
                           Matrix4d state_transition_matrix,
                           Matrix<double, 2, 4> state_to_measurement_matrix,
                           Matrix4d sys_nois_covariance,
                           Matrix2d measurement_covariance)
{
  this->state_estimate = state_estimate;
  this->state_covar = state_covar;
  this->state_transition_matrix = state_transition_matrix;
  this->state_to_measurement_matrix = state_to_measurement_matrix;
  this->sys_nois_covariance = sys_nois_covariance;
  this->measurement_covariance = measurement_covariance;
}

std::tuple<Vector4d, Matrix4d> KalmanFilter::filter(Vector2d measurement)
{
  Vector4d new_state_estimate = state_transition_matrix * state_estimate;
  Matrix4d new_state_covar = state_transition_matrix * state_covar *
                                 state_transition_matrix.transpose() +
                             sys_nois_covariance;
  Matrix2d innovation = state_to_measurement_matrix * new_state_covar *
                            state_to_measurement_matrix.transpose() +
                        measurement_covariance;
  Matrix<double, 4, 2> kalman_gain = new_state_covar *
                                     state_to_measurement_matrix.transpose() *
                                     innovation.inverse();

  Vector2d residual =
      measurement - state_to_measurement_matrix * new_state_estimate;
  state_estimate = new_state_estimate + kalman_gain * residual;
  state_covar = new_state_covar -
                kalman_gain * state_to_measurement_matrix * new_state_covar;
  return {state_estimate, state_covar};
}

// int main() {
//   srand(time(NULL));
//   Vector2d measurement(0, 0);
//   double dt = 0.1;

//   Vector4d state_estimate{0, 0, 20, 20};
//   Matrix4d state_covar{{5, 0, 0, 0}, {0, 5, 0, 0}, {0, 0, 5, 0}, {0, 0, 0, 0}};

//   Matrix4d state_transition_matrix{
//       {1, 0, dt, 0},
//       {0, 1, 0, dt},
//       {0, 0, 1, 0},
//       {0, 0, 0, 1},
//   };
//   Matrix<double, 2, 4> state_to_measurement_matrix;
//   state_to_measurement_matrix << 1, 0, 0, 0, 0, 1, 0, 0;

//   Matrix4d sys_nois_covariance{
//       {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 3, 0}, {0, 0, 0, 3}};

//   Matrix2d measurement_covar{{10, 0}, {0, 10}};

//   KalmanFilter kf(state_estimate, state_covar, measurement,
//                   state_transition_matrix, state_to_measurement_matrix,
//                   sys_nois_covariance, measurement_covar);
//   for (int i = 0; i < 300; i++) {
//     Vector2d measurement = kf.GetMeasurement();
//     auto [state_estimate, state_covar] = kf.filter(measurement);
//   }
// }

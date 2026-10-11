#include "../Eigen/Dense" // using Eigen 5
#include <tuple>
using namespace Eigen;

class KalmanFilter
{
public:
  KalmanFilter(Vector4d state_estimate, Matrix4d state_covar, Matrix4d state_transition_matrix,
               Matrix<double, 2, 4> state_to_measurement_matrix,
               Matrix4d sys_nois_covariance, Matrix2d measurement_covariance);

  std::tuple<Vector4d, Matrix4d> filter(Vector2d measurement);

private:
  Vector4d state_estimate;                          // 4 x 1 - x
  Matrix4d state_covar;                             // 4 x 4 - P
  Vector2d measurement;                             // (x, y)
  Matrix4d state_transition_matrix;                 // 4x4 A
  Matrix<double, 2, 4> state_to_measurement_matrix; // H 2 x 4
  Matrix4d sys_nois_covariance;                     // Q 4 x 4
  Matrix2d measurement_covariance;                  // R 2x2
};

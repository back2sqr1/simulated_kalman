#include "filter_interface/srv/state_estimate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kalman-filter-no-ros/kalman_filter.h"

class KalmanFilterService : public rclcpp::Node
{
public:
    KalmanFilterService(KalmanFilter KalmanFilter)
        : Node("kalman_filter_service"), kalman_filter_(KalmanFilter)
    {
        service_ = this->create_service<filter_interface::srv::StateEstimate>(
            "state_estimate",
            std::bind(&KalmanFilterService::filter, this, std::placeholders::_1, std::placeholders::_2));
    }

    void filter(const std::shared_ptr<filter_interface::srv::StateEstimate::Request> request,
                std::shared_ptr<filter_interface::srv::StateEstimate::Response> response)
    {
        Vector2d measurement(request->measurement.x, request->measurement.y);
        auto [state_estimate, state_covar] = kalman_filter_.filter(measurement);
        response->state_estimate.x = state_estimate(0);
        response->state_estimate.y = state_estimate(1);
        response->state_estimate.vx = state_estimate(2);
        response->state_estimate.vy = state_estimate(3);
        RCLCPP_INFO(this->get_logger(), "Received measurement: (%f, %f)", request->measurement.x, request->measurement.y);
        RCLCPP_INFO(this->get_logger(), "State estimate: (%f, %f, %f, %f)", state_estimate(0), state_estimate(1), state_estimate(2), state_estimate(3));
    }

private:
    rclcpp::Service<filter_interface::srv::StateEstimate>::SharedPtr service_;
    KalmanFilter kalman_filter_;
};

int main(int argc, char **argv)
{
    srand(time(NULL));
    double dt = 0.1;

    Vector4d state_estimate{0, 0, 20, 20};
    Matrix4d state_covar{{5, 0, 0, 0}, {0, 5, 0, 0}, {0, 0, 5, 0}, {0, 0, 0, 0}};

    Matrix4d state_transition_matrix{
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    Matrix<double, 2, 4> state_to_measurement_matrix;
    state_to_measurement_matrix << 1, 0, 0, 0, 0, 1, 0, 0;

    Matrix4d sys_nois_covariance{
        {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 3, 0}, {0, 0, 0, 3}};

    Matrix2d measurement_covar{{10, 0}, {0, 10}};

    KalmanFilter kf(state_estimate, state_covar,
                    state_transition_matrix, state_to_measurement_matrix,
                    sys_nois_covariance, measurement_covar);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterService>(kf));
    rclcpp::shutdown();
    return 0;
}
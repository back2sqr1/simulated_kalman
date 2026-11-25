#include "filter_interface/msg/filter.hpp"
#include "filter_interface/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kalman-filter-no-ros/kalman_filter.h"
#include <random>

// Gazebo publishes the robot's true position. This node adds noise to it
// (a fake GPS), runs the Kalman filter on that, and publishes the estimate.
class KalmanFilterGazebo : public rclcpp::Node
{
public:
    KalmanFilterGazebo(KalmanFilter kalman_filter)
        : Node("kalman_filter_gazebo"), kalman_filter_(kalman_filter)
    {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/robot/odometry", 10,
            std::bind(&KalmanFilterGazebo::filter, this, std::placeholders::_1));
        measurement_pub_ = this->create_publisher<filter_interface::msg::Point>("measurement", 10);
        estimate_pub_ = this->create_publisher<filter_interface::msg::Filter>("estimate", 10);
    }

private:
    void filter(const nav_msgs::msg::Odometry &odometry)
    {
        double true_x = odometry.pose.pose.position.x;
        double true_y = odometry.pose.pose.position.y;
        Vector2d measurement(true_x + noise_(gen_), true_y + noise_(gen_));
        auto [state_estimate, state_covar] = kalman_filter_.filter(measurement);

        filter_interface::msg::Point measurement_msg;
        measurement_msg.x = measurement(0);
        measurement_msg.y = measurement(1);
        measurement_pub_->publish(measurement_msg);

        filter_interface::msg::Filter estimate_msg;
        estimate_msg.x = state_estimate(0);
        estimate_msg.y = state_estimate(1);
        estimate_msg.vx = state_estimate(2);
        estimate_msg.vy = state_estimate(3);
        estimate_pub_->publish(estimate_msg);

        RCLCPP_INFO(this->get_logger(),
                    "true (%.2f, %.2f)  measured (%.2f, %.2f)  estimate (%.2f, %.2f)  velocity (%.2f, %.2f)",
                    true_x, true_y, measurement(0), measurement(1),
                    state_estimate(0), state_estimate(1), state_estimate(2), state_estimate(3));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<filter_interface::msg::Point>::SharedPtr measurement_pub_;
    rclcpp::Publisher<filter_interface::msg::Filter>::SharedPtr estimate_pub_;
    KalmanFilter kalman_filter_;
    std::mt19937 gen_{std::random_device{}()};
    std::normal_distribution<double> noise_{0.0, 0.5}; // std dev 0.5 m
};

int main(int argc, char **argv)
{
    double dt = 0.1; // worlds/kalman.sdf publishes odometry at 10 Hz

    Vector4d state_estimate{0, 0, 0, 0};
    Matrix4d state_covar{{5, 0, 0, 0}, {0, 5, 0, 0}, {0, 0, 5, 0}, {0, 0, 0, 5}};

    Matrix4d state_transition_matrix{
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };
    Matrix<double, 2, 4> state_to_measurement_matrix;
    state_to_measurement_matrix << 1, 0, 0, 0, 0, 1, 0, 0;

    Matrix4d sys_nois_covariance{
        {0.001, 0, 0, 0}, {0, 0.001, 0, 0}, {0, 0, 0.01, 0}, {0, 0, 0, 0.01}};

    Matrix2d measurement_covar{{0.25, 0}, {0, 0.25}}; // 0.5^2, the noise added above

    KalmanFilter kf(state_estimate, state_covar,
                    state_transition_matrix, state_to_measurement_matrix,
                    sys_nois_covariance, measurement_covar);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterGazebo>(kf));
    rclcpp::shutdown();
    return 0;
}

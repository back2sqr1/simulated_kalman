#include "rclcpp/rclcpp.hpp"
#include "filter_interface/srv/state_estimate.hpp"
#include "Eigen/Dense"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

double drand(double start, double end)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the range [5.5, 10.5)
    std::uniform_real_distribution<double> dis(start, end);
    return dis(gen);
}


// moves and get measurement
Eigen::Vector2d GetMeasurement(Eigen::Vector2d& true_position)
{
    double dt = 0.1;
    Eigen::Vector2d measurement(true_position(0), true_position(1));
    Eigen::Vector2d noise(drand(-5.0, 5.0), drand(-5.0, 5.0));
    measurement += noise; // add some noise to the measurement

    Eigen::Vector2d velocity(60, 60);
    for (int i = 0; i < 2; i++)
    {
        velocity(i) += drand(-5.0, 5.0);
        // vx or vy with some error
        measurement(i) += velocity(i) * dt + drand(-1.0, 1.0);
        // change in dx or dy with some error
    }
    true_position(0) += velocity(0) * dt;
    true_position(1) += velocity(1) * dt;
    return measurement;
}

class KalmanFilterClient : public rclcpp::Node
{
public:
    KalmanFilterClient()
        : Node("kalman_filter_client")
    {
        client_ = this->create_client<filter_interface::srv::StateEstimate>("state_estimate");
        timer_ = this->create_wall_timer(100ms, std::bind(&KalmanFilterClient::send_request, this));
        true_position << 0, 0;
    }

private:
    void send_request()
    {
        auto request = std::make_shared<filter_interface::srv::StateEstimate::Request>();
        // Set the request data
        Eigen::Vector2d measurement = GetMeasurement(true_position);
        request->measurement.x = measurement(0);
        request->measurement.y = measurement(1);
        RCLCPP_INFO(this->get_logger(), "Sending measurement: (%f, %f)", request->measurement.x, request->measurement.y);
        // Send the request asynchronously

        client_->async_send_request(request);
    }
    rclcpp::Client<filter_interface::srv::StateEstimate>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Vector2d true_position;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
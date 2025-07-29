#pragma once

#include "ikfom_formulation.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"


class ImuIkfomNode final : public rclcpp::Node {
public:
    ImuIkfomNode() : Node("imu_ikfom_node") {
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                on_imu_msg(msg);
            }
        );

        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_filtered", 10);

        param_ikfom_eps = this->declare_parameter("ikfom_eps", 1e-6);
        param_ikfom_max_iter = this->declare_parameter("ikfom_max_iter", 1000);
        param_ang_vel_var = this->declare_parameter("ang_vel_var", 0.001);
        param_lin_acc_var = this->declare_parameter("lin_acc_var", 0.01);
        param_enable_runtime_printing = this->declare_parameter("enable_runtime_printing", false);

        estimator = new esekfom::esekf<state, 3, input, measurement, 3>();
        double eps[3];
        std::fill(std::begin(eps), std::end(eps), param_ikfom_eps);
        estimator->init(f, df_dx, df_dw, h, dh_dx, dh_dv, param_ikfom_max_iter, eps);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    double param_ikfom_eps{1e-6}, param_ang_vel_var{0.01}, param_lin_acc_var{0.01};
    int param_ikfom_max_iter{1000};
    bool param_enable_runtime_printing{false};

    esekfom::esekf<state, 3, input, measurement, 3> *estimator{nullptr};

    rclcpp::Time last_msg_t{};
    bool is_first_msg{true};

    void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (is_first_msg) {
            last_msg_t = msg->header.stamp;
            is_first_msg = false;
            return;
        }

        // get dt
        rclcpp::Time t = msg->header.stamp;
        double dt = (t - last_msg_t).seconds();
        last_msg_t = t;

        // estimator prediction
        input u;
        u.ang_vel << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        auto process_noise_cov = mat_Q(param_ang_vel_var);
        auto start_predict = std::chrono::high_resolution_clock::now();
        estimator->predict(dt, process_noise_cov, u);
        auto end_predict = std::chrono::high_resolution_clock::now();


        // estimator update
        measurement z;
        z.lin_acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        auto measurement_noise_cov = mat_R(param_lin_acc_var);
        auto start_update = std::chrono::high_resolution_clock::now();
        estimator->update_iterated(z, measurement_noise_cov);
        auto end_update = std::chrono::high_resolution_clock::now();

        // get estimation
        auto x = estimator->get_x();
        auto mat_P = estimator->get_P();
        auto q = Eigen::Quaterniond(x.rot.toRotationMatrix());

        // publish values
        sensor_msgs::msg::Imu imu_msg(*msg);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        imu_pub->publish(imu_msg);

        if (param_enable_runtime_printing) {
            std::chrono::duration<double, std::milli> elapsed_predict = end_predict - start_predict;
            std::chrono::duration<double, std::milli> elapsed_update = end_update - start_update;
            RCLCPP_INFO(this->get_logger(), "predict: %f, update %f", elapsed_predict.count(), elapsed_update.count());
        }
    }
};

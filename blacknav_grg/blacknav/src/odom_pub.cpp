#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

constexpr double PI = 3.141592653;

constexpr double TRACK_WIDTH = 117.00 * 0.001;
constexpr double WHEEL_RADIUS = 33.25 * 0.001;

constexpr unsigned MOTOR_GEAR_RATIO = 120;
constexpr unsigned ENCODER_TICKS_PER_ROTATION = 6;
constexpr unsigned MOTOR_TICKS_PER_DEGREE = ((MOTOR_GEAR_RATIO * ENCODER_TICKS_PER_ROTATION) / 360.0);

constexpr unsigned SPI_ADDR = 8;

constexpr unsigned LEFT_MOTOR = 0;
constexpr unsigned RIGHT_MOTOR = 1;

constexpr unsigned SPI_GET_MOTOR_ENCODER_LEFT = 17;
constexpr unsigned SPI_GET_MOTOR_ENCODER_RIGHT = 18;

int pigpio_handle = 0;
int spi_handle = 0;

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher()
    : Node("odom_pub") {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pigpio_handle = pigpio_start(nullptr, nullptr);
        if (pigpio_handle < 0) {
            throw std::runtime_error("Can't start pigpio, exiting...");
        }

        spi_handle = spi_open(pigpio_handle, 1, 100000, 0);
        if (spi_handle < 0) {
            throw std::runtime_error("Can't start SPI Bus, exiting...");
        }

        set_mode(pigpio_handle, 9, PI_ALT0);
        set_mode(pigpio_handle, 10, PI_ALT0);
        set_mode(pigpio_handle, 11, PI_ALT0);
        set_mode(pigpio_handle, 23, PI_OUTPUT);

        gpio_write(pigpio_handle, 23, 1);

        initial_time = get_current_tick(pigpio_handle);
        final_time = get_current_tick(pigpio_handle);

        final_left_enc = get_encoder_radians(LEFT_MOTOR);
        initial_left_enc = final_left_enc;

        rclcpp::sleep_for(1ms);

        final_right_enc = get_encoder_radians(RIGHT_MOTOR);
        initial_right_enc = final_right_enc;

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        RCLCPP_INFO(this->get_logger(), "Starting up pose publisher.");
        auto odom_callback =
            [this]() -> void {
                nav_msgs::msg::Odometry odom = this->calculate_odom();
                geometry_msgs::msg::TransformStamped transform = calculate_transform(odom);

                this->publisher_->publish(odom);
                this->tf2_broadcaster_->sendTransform(transform);
            };

        timer_ = this->create_wall_timer(200ms, odom_callback);
    }

    ~OdomPublisher() {
        spi_close(pigpio_handle, spi_handle);
        pigpio_stop(pigpio_handle);
    }
    
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    int pigpio_handle;
    int spi_handle;

    unsigned initial_time;
    unsigned final_time;

    double initial_left_enc;
    double final_left_enc;

    double initial_right_enc;
    double final_right_enc;

    double x;
    double y;
    double theta;

    double get_encoder_radians(unsigned motor) {
        unsigned message_type = 0;

        if(motor == LEFT_MOTOR) {
            message_type = SPI_GET_MOTOR_ENCODER_LEFT;
        } else if(motor == RIGHT_MOTOR) {
            message_type = SPI_GET_MOTOR_ENCODER_RIGHT;
        }

        std::array<char, 8> encoder_out = {
            SPI_ADDR,
            message_type,
            0,
            0,
            0,
            0,
            0,
            0,
        };
        std::array<char, 8> encoder_reply{};

        int encoder_degrees = 0;
        double encoder_radians = 0.0;

        spi_xfer(pigpio_handle, spi_handle, encoder_out.data(), encoder_reply.data(), sizeof(encoder_reply.data()));
        if (encoder_reply[3] == 0xA5) {
            encoder_degrees = static_cast<int>(
                encoder_reply[4] << 24 
                | encoder_reply[5] << 16 
                | encoder_reply[6] << 8 
                | encoder_reply[7]
            );

            if(encoder_degrees & 0x80000000) {
                encoder_degrees -= 0x100000000;
            }

            encoder_degrees /= MOTOR_TICKS_PER_DEGREE;
            encoder_radians = encoder_degrees * (PI / 180);

            return encoder_radians;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "No SPI Response.");
            return 0.0;
        }
    }

    nav_msgs::msg::Odometry calculate_odom() {
        nav_msgs::msg::Odometry odom;

        initial_time = final_time;
        final_time = get_current_tick(pigpio_handle);

        initial_left_enc = final_left_enc;
        initial_right_enc = final_right_enc;

        double final_left_enc_ = get_encoder_radians(LEFT_MOTOR);
        if(final_left_enc_ != 0.0) {
            final_left_enc = final_left_enc_;
        }
        
        rclcpp::sleep_for(5ms);

        double final_right_enc_ = get_encoder_radians(RIGHT_MOTOR);
        if(final_right_enc_ != 0.0) {
            final_right_enc = final_right_enc_;
        }

        unsigned dt = final_time - initial_time;
        double lm_speed = (final_left_enc - initial_left_enc) / (dt * std::pow(10, -6));
        double rm_speed = (final_right_enc - initial_right_enc) / (dt * std::pow(10, -6));

        double r = WHEEL_RADIUS;
        double s = TRACK_WIDTH;

        double angular_speed = (r/s) * (rm_speed - lm_speed);
        double linear_speed = (rm_speed + lm_speed) * (r/2);
        
        double x_speed = linear_speed * std::cos(theta);
        double y_speed = linear_speed * std::sin(theta);
        double theta_speed = angular_speed;

        x += x_speed * (dt * std::pow(10, -6));
        y += y_speed * (dt * std::pow(10, -6));
        theta += theta_speed * (dt * std::pow(10, -6));

        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);

        std::array<double, 36> covariance = {0};

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.w = quat.getW();
        odom.pose.pose.orientation.x = quat.getX();
        odom.pose.pose.orientation.y = quat.getY();
        odom.pose.pose.orientation.z = quat.getZ();

        odom.twist.twist.linear.x = linear_speed;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = angular_speed;

        odom.pose.covariance = covariance;
        odom.twist.covariance = covariance;

        odom.header.stamp = rclcpp::Time(static_cast<uint64_t>(time_time() * std::pow(10, 9)));
        odom.header.frame_id = "odom";

        odom.child_frame_id = "main_body";

        return odom;
    }

    geometry_msgs::msg::TransformStamped calculate_transform(nav_msgs::msg::Odometry odom) {
        geometry_msgs::msg::TransformStamped transform;

        transform.header = odom.header;
        transform.child_frame_id = odom.child_frame_id;

        transform.transform.translation.x = odom.pose.pose.position.x;
        transform.transform.translation.y = odom.pose.pose.position.y;
        transform.transform.translation.z = odom.pose.pose.position.z;

        transform.transform.rotation.w = odom.pose.pose.orientation.w;
        transform.transform.rotation.x = odom.pose.pose.orientation.x;
        transform.transform.rotation.y = odom.pose.pose.orientation.y;
        transform.transform.rotation.z = odom.pose.pose.orientation.z;

        return transform;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();

    return 0;
}
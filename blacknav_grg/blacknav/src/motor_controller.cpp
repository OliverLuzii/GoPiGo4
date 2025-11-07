#define _USE_MATH_DEFINES

#include <cmath> 
#include <array>
#include <memory>
#include <iostream>
#include <stdexcept>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

constexpr float TRACK_WIDTH = 117.00 * 0.001;
constexpr float WHEEL_RADIUS = 33.25 * 0.001;

constexpr unsigned MOTOR_GEAR_RATIO = 120;
constexpr unsigned ENCODER_TICKS_PER_ROTATION = 6;
constexpr float MOTOR_TICKS_PER_DEGREE = (MOTOR_GEAR_RATIO * ENCODER_TICKS_PER_ROTATION) / 360.0;

constexpr unsigned MOTOR_LEFT = 0x01;
constexpr unsigned MOTOR_RIGHT = 0x02;

constexpr unsigned SPI_ADDR = 8;
constexpr unsigned SPI_MOTOR_DPS = 14;

using std::placeholders::_1;

int pigpio_handle = 0;
int spi_handle = 0;

class TwistSubscriber : public rclcpp::Node {
public:
    TwistSubscriber()
    : Node("twist_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", // Topic name
            10,        // QoS history depth
            std::bind(&TwistSubscriber::twist_callback, this, std::placeholders::_1));

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

    }

    ~TwistSubscriber() {
        spi_close(pigpio_handle, spi_handle);
        pigpio_stop(pigpio_handle);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), 
            "Linear Velocity: %.2f m/s, Angular Velocity: %.2f rad/s",
            msg->linear.x, msg->angular.z);
        
        // Inverse kinematics and conversion from rad/s to deg/s
        int theta_dot_L = static_cast<int>(std::round((msg->linear.x / WHEEL_RADIUS - msg->angular.z * TRACK_WIDTH / (2.0 * WHEEL_RADIUS)) * (180.0 / M_PI) * MOTOR_TICKS_PER_DEGREE));
        int theta_dot_R = static_cast<int>(std::round((msg->linear.x / WHEEL_RADIUS + msg->angular.z * TRACK_WIDTH / (2.0 * WHEEL_RADIUS)) * (180.0 / M_PI) * MOTOR_TICKS_PER_DEGREE));

        std::array<char, 5> message_L = {
            SPI_ADDR,
            SPI_MOTOR_DPS,
            MOTOR_LEFT,
            ((theta_dot_L >> 8) & 0xFF),
            (theta_dot_L & 0xFF)
        };

        std::array<char, 5> message_R = {
            SPI_ADDR,
            SPI_MOTOR_DPS,
            MOTOR_RIGHT,
            ((theta_dot_R >> 8) & 0xFF),
            (theta_dot_R  & 0xFF)
        };

        spi_write(pigpio_handle, spi_handle, message_L.data(), sizeof(message_L));
        spi_write(pigpio_handle, spi_handle, message_R.data(), sizeof(message_R));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
    // Configuring gpio shit for spi comm
    pigpio_handle = pigpio_start(nullptr, nullptr);
    if(pigpio_handle < 0) {
        std::cout << "Can't start pigpio, exiting..." << std::endl;
        return 1;
    }
    
    spi_handle = spi_open(pigpio_handle, 1, 100000, 0);
    if(spi_handle < 0) {
        std::cout << "Can't start SPI Bus, exiting..." << std::endl;
        return 1;
    }

    set_mode(pigpio_handle, 9, PI_ALT0);
    set_mode(pigpio_handle, 10, PI_ALT0);
    set_mode(pigpio_handle, 11, PI_ALT0);
    set_mode(pigpio_handle, 23, PI_OUTPUT);

    gpio_write(pigpio_handle, 23, 1);

    // Start node, let's fucking go
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistSubscriber>());
    rclcpp::shutdown();

    return 0;
}

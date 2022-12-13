#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <front_milli_wave_sensor_msg/msg/tracker_array.hpp>

class OSA79GAL : public rclcpp::Node {
    public:
        OSA79GAL();
        ~OSA79GAL();

    private:
        /* for ros */
        void timerCallback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<front_milli_wave_sensor_msg::msg::TrackerArray>::SharedPtr tracker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        /* for string */
        std::string getSubString(std::string& data, const size_t begin);
        std::string getSubString(std::string& data, const size_t begin, const size_t end);

        /** geometric info */
        std::string frame_;
        geometry_msgs::msg::Pose pose_;

        /* for uart communication */
        std::string device_name_;
        int openSerial(const char* _device_name);
        void setupSensor();

        int fd1_;
        const std::string magic_word_ = "GTRACKER";
        const std::string platform_ = "AR77";
        const size_t point_size_ = 51;

        typedef struct {
            int ind;
            int size;
        } component;

        struct {
            component id = {1, 3};
            component x = {5, 4};
            component y = {10, 5};
            component vx = {17, 4};
            component vy = {13, 5};
            component ax = {30, 4};
            component ay = {36, 5};
            component gain = {42, 4};
            component power = {47, 4};
        } index;
};
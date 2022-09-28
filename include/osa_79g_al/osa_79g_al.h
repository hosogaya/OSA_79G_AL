#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class OSA79GAL : public rclcpp::Node {
    public:
        OSA79GAL(const char* _device_name);

    private:
        /* for ros */
        void timerCallback();
        rclcpp::TimerBase::SharedPtr timer_;

        /* for uart communication */
        int openSerial(const char* _device_name);
        void setupSensor();

        int fd1_;
        const std::string magic_word_ = "GTRACKER";
        const std::string platform_ = "AR77";
        const int point_size_ = 51;

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
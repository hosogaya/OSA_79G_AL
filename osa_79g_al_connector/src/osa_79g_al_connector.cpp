/**
 * @file osa_79g_al_connector.cpp
 * @author Hirotaka Hosogaya (hosogaya.hirotaka.z7@s.mail.nagoya-u.ac.jp)
 * @brief This code retrieve data from OSA-79G-AL (https://www.akasakatec.com/products/hardware/mm-wave-radar-module/).
 *        This code is written by reference to  https://qiita.com/srs/items/efaa8dc0a6d580c7c423 .
 * @version 0.1
 * @date 2022-09-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <osa_79g_al_connector/osa_79g_al_connector.h>


// To simplify timer definition
using namespace std::chrono_literals;

OSA79GAL::OSA79GAL(const char* _device_name) : Node("osa_79g_al")
{
    fd1_ = this->openSerial(_device_name);
    if (fd1_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed: could not open %s", _device_name);
        exit(1);
    }
    publisher_ = this->create_publisher<tracker_msg::msg::Tracker>(
        "/osa_79g_al_tracker", 1
    );
    timer_ = this->create_wall_timer(
        100ms, std::bind(&OSA79GAL::timerCallback, this)
    );
}

int OSA79GAL::openSerial(const char* _device_name) {    
    int fd1 = open(_device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL, 0);

    // load configuration
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);
    // set baudrate
    speed_t BAUDRATE = B921600;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    // non canonical, no echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    // non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    // store configuration
    tcsetattr(fd1, TCSANOW, &conf_tio);
    return fd1;
}

void OSA79GAL::setupSensor() {
    /* send start command */

    /* send setting x,y */
}

void OSA79GAL::timerCallback() 
{
    char buff[256] = {0};
    int recv_data = read(fd1_, buff, sizeof(buff));
    RCLCPP_INFO(this->get_logger(), "Received %d data", recv_data);
    if (recv_data > 0) {
        std::string data = buff;
        // find magic word
        int m = data.find(magic_word_);
        if (m == std::string::npos) return;
        else data = data.substr(m+magic_word_.size());
        // find platform 
        int p = data.find(platform_);
        if (p == std::string::npos) return;
        else data = data.substr(p + platform_.size());

        // get tracker num
        int num = stoi(data.substr(1, 2));
        if (num < 0) return;
        else RCLCPP_INFO(this->get_logger(), "Get %d trackers.", num);

        // create container
        
        // substitute data
        data = data.substr(2);
        std::string d;
        for (int i = 0; i < num; ++i) {
            d = data.substr(point_size_*i, point_size_); // retrieve point
            int id = stoi(d.substr(index.id.ind, index.id.size));
            float x = static_cast<float>(stoi(d.substr(index.x.ind, index.x.size)))*0.1f;
            float y = static_cast<float>(stoi(d.substr(index.y.ind, index.y.size)))*0.1f;
            float vx = static_cast<float>(stoi(d.substr(index.vx.ind, index.vx.size)))*0.1f;
            float vy = static_cast<float>(stoi(d.substr(index.vy.ind, index.vy.size)))*0.1f;
            float ax = static_cast<float>(stoi(d.substr(index.ax.ind, index.ax.size)))*0.1f;
            float ay = static_cast<float>(stoi(d.substr(index.ay.ind, index.ay.size)))*0.1f;
            float gain = static_cast<float>(stoi(d.substr(index.gain.ind, index.gain.size)))*0.1f;
            float power = static_cast<float>(stoi(d.substr(index.power.ind, index.power.size)))*0.1f;
        
            RCLCPP_INFO(this->get_logger(), "id: %d, x: %f, y: %f, vx: %f, vy: %f, ax: %f, ay: %f, gain: %f, power: %f", id, x, y, vx, vy, ax, ay, gain, power);
        }
    }
}
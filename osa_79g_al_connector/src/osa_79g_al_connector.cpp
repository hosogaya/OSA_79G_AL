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

OSA79GAL::OSA79GAL() : Node("osa_79g_al")
{
    declare_parameter("x_range", "20.0");
    declare_parameter("y_range", "30.0");
    declare_parameter("device_name", "/dev/ttyUSB0");

    std::string command_x = "stx " + get_parameter("x_range").as_string() + "\n";
    std::string command_y = "sty " + get_parameter("y_range").as_string() + "\n";
    device_name_ = get_parameter("device_name").as_string();
    RCLCPP_INFO(this->get_logger(), "x range: %s", command_x.c_str());
    RCLCPP_INFO(this->get_logger(), "y range: %s", command_y.c_str());
    RCLCPP_INFO(this->get_logger(), "device_name: %s", device_name_.c_str());

    /* open serial port */
    fd1_ = this->openSerial(device_name_.c_str());
    if (fd1_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed: could not open %s", device_name_.c_str());
        exit(1);
    }
    setupSensor();

    publisher_ = this->create_publisher<tracker_array_msg::msg::TrackerArray>(
        "/osa_79g_al_trackers", 1
    );
    timer_ = this->create_wall_timer(
        100ms, std::bind(&OSA79GAL::timerCallback, this)
    );
}

OSA79GAL::~OSA79GAL() {
    char command[] = "p";
    write(fd1_, command, sizeof(command));
    close(fd1_);
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
    // send command to start uart communication
    char command[] = "ar 1 2 0\n";
    write(fd1_, command, sizeof(command));
    /* send setting x,y */
    std::string command_x = "stx " + get_parameter("x_range").as_string() + "\n";
    std::string command_y = "sty " + get_parameter("y_range").as_string() + "\n";
    write(fd1_, command_x.c_str(), command_x.size());
    write(fd1_, command_y.c_str(), command_y.size());
}

void OSA79GAL::timerCallback() 
{
    char buff[256] = {0};
    size_t recv_data_size = read(fd1_, buff, sizeof(buff));
    RCLCPP_INFO(this->get_logger(), "Received %d data", recv_data_size);
    if (recv_data_size > 0) {
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
        tracker_array_msg::msg::TrackerArray msg;
        msg.num = num;
        msg.data.resize(num);
        
        // substitute data
        data = data.substr(3);
        std::string d;
        for (int i = 0; i < num; ++i) {
            d = data.substr(point_size_*i, point_size_); // retrieve point
            if (d.size() != point_size_) continue; // check data size
            msg.data[i].id = stoi(d.substr(index.id.ind, index.id.size));
            msg.data[i].x = static_cast<float>(stoi(d.substr(index.x.ind, index.x.size)))*0.1f;
            msg.data[i].y = static_cast<float>(stoi(d.substr(index.y.ind, index.y.size)))*0.1f;
            msg.data[i].vx = static_cast<float>(stoi(d.substr(index.vx.ind, index.vx.size)))*0.1f;
            msg.data[i].vy = static_cast<float>(stoi(d.substr(index.vy.ind, index.vy.size)))*0.1f;
            msg.data[i].ax = static_cast<float>(stoi(d.substr(index.ax.ind, index.ax.size)))*0.1f;
            msg.data[i].ay = static_cast<float>(stoi(d.substr(index.ay.ind, index.ay.size)))*0.1f;
            float gain = static_cast<float>(stoi(d.substr(index.gain.ind, index.gain.size)))*0.1f;
            float power = static_cast<float>(stoi(d.substr(index.power.ind, index.power.size)))*0.1f;
        
            RCLCPP_INFO(this->get_logger(), "id: %d, x: %f, y: %f, vx: %f, vy: %f, ax: %f, ay: %f, gain: %f, power: %f", 
                msg.data[i].id, msg.data[i].x, msg.data[i].y, msg.data[i].vx, msg.data[i].vy, msg.data[i].ax, msg.data[i].ay, gain, power);
        }

        publisher_->publish(msg);
    }
}
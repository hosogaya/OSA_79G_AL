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

    tracker_pub_ = this->create_publisher<front_milli_wave_sensor_msg::msg::TrackerArray>(
        "/front_milli_wave_sensor_trackers", 1
    );
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "/front_milli_wave_sensor_marker_array", 1
    );
    // pub_marker_msg_
    timer_ = this->create_wall_timer(
        100ms, std::bind(&OSA79GAL::timerCallback, this)
    );
}

OSA79GAL::~OSA79GAL() {
    // char command[] = "p\n";
    // write(fd1_, command, sizeof(command));
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
    RCLCPP_INFO(this->get_logger(), "Received %d data", static_cast<int>(recv_data_size));
    if (recv_data_size >= point_size_+magic_word_.size()) {
        std::string data = buff;
        // find magic word
        size_t m = data.find(magic_word_);
        if (m == std::string::npos) return;
        else data = getSubString(data, m+magic_word_.size());
        // find platform 
        size_t p = data.find(platform_);
        if (p == std::string::npos) return;
        else data = getSubString(data, p+platform_.size());

        // get tracker num
        int num = stoi(getSubString(data, 1, 2));
        if (num <= 0) return;

        // create container
        front_milli_wave_sensor_msg::msg::TrackerArray msg;
        msg.num = num;
        msg.data.resize(num);
        

        // substitute data
        data = getSubString(data, 3);
        std::string d;
        int valid_num = num;
        std::vector<bool> valid;
        valid.resize(num);
        for (int i = 0; i < num; ++i) {
            try {
                d = data.substr(point_size_*i, point_size_); // retrieve point

                msg.data[i].id = stoi(d.substr(index.id.ind, index.id.size));
                msg.data[i].y = -static_cast<float>(stoi(d.substr(index.x.ind, index.x.size)))*0.1f; // [dm -> m]
                msg.data[i].x = static_cast<float>(stoi(d.substr(index.y.ind, index.y.size)))*0.1f;
                msg.data[i].vy = -static_cast<float>(stoi(d.substr(index.vx.ind, index.vx.size)))*0.1f;
                msg.data[i].vx = static_cast<float>(stoi(d.substr(index.vy.ind, index.vy.size)))*0.1f;
                msg.data[i].ay = -static_cast<float>(stoi(d.substr(index.ax.ind, index.ax.size)))*0.1f;
                msg.data[i].ax = static_cast<float>(stoi(d.substr(index.ay.ind, index.ay.size)))*0.1f;
                float gain = static_cast<float>(stoi(d.substr(index.gain.ind, index.gain.size)))*0.1f;
                float power = static_cast<float>(stoi(d.substr(index.power.ind, index.power.size)))*0.1f;
            
                RCLCPP_INFO(this->get_logger(), "id: %d, x: %f, y: %f, vx: %f, vy: %f, ax: %f, ay: %f, gain: %f, power: %f", 
                    msg.data[i].id, msg.data[i].x, msg.data[i].y, msg.data[i].vx, msg.data[i].vy, msg.data[i].ax, msg.data[i].ay, gain, power);

                valid[i] = true;
            }
            catch (const std::invalid_argument& e)
            {
                RCLCPP_INFO(this->get_logger(), e.what());
                valid[i] = false;
                --valid_num;
            }
            catch (const std::out_of_range& e) {
                RCLCPP_INFO(this->get_logger(), e.what());
                valid[i] = false;
                --valid_num;
            }
        }

        int ind = 0;
        float radius = 0.4f;
        front_milli_wave_sensor_msg::msg::TrackerArray  valid_msg;
        visualization_msgs::msg::MarkerArray marker_array_msg;
        if (valid_num <= 0) return;
        valid_msg.num = valid_num;
        valid_msg.data.resize(valid_num);
        marker_array_msg.markers.resize(valid_num);

        for (int i=0; i<valid_num; ++i) {
            if (!valid[i]) continue;
            
            valid_msg.data[ind] = msg.data[i];
            marker_array_msg.markers[ind].header.frame_id = "map";
            marker_array_msg.markers[ind].header.stamp = this->now();
            marker_array_msg.markers[ind].ns = "front_milli_wave_sensor";
            marker_array_msg.markers[ind].id = msg.data[i].id;
            marker_array_msg.markers[ind].type = visualization_msgs::msg::Marker::CYLINDER;
            marker_array_msg.markers[ind].action = visualization_msgs::msg::Marker::ADD;
            marker_array_msg.markers[ind].pose.position.x = msg.data[i].x;
            marker_array_msg.markers[ind].pose.position.y = msg.data[i].y;
            marker_array_msg.markers[ind].pose.position.z = 0.5f;
            marker_array_msg.markers[ind].pose.orientation.x = 0.0f;
            marker_array_msg.markers[ind].pose.orientation.y = 0.0f;
            marker_array_msg.markers[ind].pose.orientation.z = 0.0f;
            marker_array_msg.markers[ind].pose.orientation.w = 1.0f;
            marker_array_msg.markers[ind].scale.x = radius;
            marker_array_msg.markers[ind].scale.y = radius;
            marker_array_msg.markers[ind].scale.z = 1.0f;
            marker_array_msg.markers[ind].color.a = 0.5f;
            marker_array_msg.markers[ind].color.r = 1.0f;
            marker_array_msg.markers[ind].color.g = 0.0f;
            marker_array_msg.markers[ind].color.b = 0.0f;
            marker_array_msg.markers[ind].lifetime.nanosec = 100e6;
            ++ind;
        }
        RCLCPP_INFO(this->get_logger(), " send %ld data", marker_array_msg.markers.size());
        tracker_pub_->publish(valid_msg);
        marker_pub_->publish(marker_array_msg);
    }// end if (recv_data_size > 0)
}

std::string OSA79GAL::getSubString(std::string& data, const size_t begin) {
    try
    {
        std::string d = data.substr(begin);
        return d;
    }
    catch (const std::invalid_argument& e)
    {
        RCLCPP_INFO(this->get_logger(), e.what());
        return "";
    }
    catch (const std::out_of_range& e) {
        RCLCPP_INFO(this->get_logger(), e.what());
        return "";
    }
    return "";
}

std::string OSA79GAL::getSubString(std::string& data, const size_t begin, const size_t end) {
    try
    {
        std::string d = data.substr(begin, end);
        return d;
    }
    catch (const std::invalid_argument& e)
    {
        RCLCPP_INFO(this->get_logger(), e.what());
        return "";
    }
    catch (const std::out_of_range& e) {
        RCLCPP_INFO(this->get_logger(), e.what());
        return "";
    }
    return "";
}
#include <front_milli_wave_sensor_connector/front_milli_wave_sensor_connector.h>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    {
        rclcpp::spin(std::make_shared<OSA79GAL>());
    }
    rclcpp::shutdown();

    return 0;
}
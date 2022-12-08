#include <osa_79g_al_connector/osa_79g_al_connector.h>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OSA79GAL>());
    rclcpp::shutdown();

    return 0;
}
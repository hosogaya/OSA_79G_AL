#include <osa_79g_al/osa_79g_al.h>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OSA79GAL>(argv[1]));
    rclcpp::shutdown();

    return 0;
}
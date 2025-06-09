#include <rclcpp/rclcpp.hpp>

#include "detector/filter.h"

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Filter>());
   rclcpp::shutdown();
   return 0;
}

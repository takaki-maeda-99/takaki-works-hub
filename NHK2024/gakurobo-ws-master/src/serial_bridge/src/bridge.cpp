#include "serial_bridge/bridge.h"

Bridge::Bridge() : Node("bridge") {
   lidar_condition_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/lidar_condition", 10);
   camera_condition_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/camera_condition", 10);
   obstacles_subscription_ = this->create_subscription<obstacle_msgs::msg::Obstacles>("/obstacles", 10, std::bind(&Bridge::obstaclesCallback, this, std::placeholders::_1));
   ball_camera_subscription_ = this->create_subscription<ball_camera_msgs::msg::Balls>("/ball_camera", 10, std::bind(&Bridge::ballCameraCallback, this, std::placeholders::_1));

   port_ = declare_parameter("port", "/dev/ttyACM0");
   baudrate_ = declare_parameter("baudrate", 115200);
   debug_ = declare_parameter("debug", false);
   interval_ms_ = declare_parameter("interval_ms", 200);

   timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms_), std::bind(&Bridge::timerCallback, this));

   serial_.setBaudrate(baudrate_);
   serial_.setPort(port_);
   serial_.open();

   if (serial_.isOpen()) {
      RCLCPP_ERROR(this->get_logger(), "serial port %s %s", port_.c_str(), "opened");
   } else {
      RCLCPP_ERROR(this->get_logger(), "serial port %s %s", port_.c_str(), "not found");
   }

   MsgPacketizer::publish(serial_, 0x01, tx_msg_)->setFrameRate(20);
   if (debug_) {
      MsgPacketizer::subscribe(serial_, 0x02, rx_msg_);
   }
}

void Bridge::obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg) {
   if (msg->circles.size() > 0) {
      tx_msg_.nearest_circle.center_x = msg->circles[0].center.x;
      tx_msg_.nearest_circle.center_y = msg->circles[0].center.y;
      tx_msg_.nearest_circle.radius = msg->circles[0].radius;
   } else {
      tx_msg_.nearest_circle.center_x = 0.0;
      tx_msg_.nearest_circle.center_y = 0.0;
      tx_msg_.nearest_circle.radius = 0.0;
   }

   size_t i = 0;
   for (; i < msg->walls.size(); ++i) {
      tx_msg_.walls[i].direction = msg->walls[i].angle;
      tx_msg_.walls[i].distance = msg->walls[i].distance;
   }
   for (; i < 4; ++i) {
      tx_msg_.walls[i].direction = 0.0;
      tx_msg_.walls[i].distance = 0.0;
   }
}

void Bridge::ballCameraCallback(const ball_camera_msgs::msg::Balls::SharedPtr msg) {
   if (msg->balls.size() > 0) {
      ball_camera_msgs::msg::Ball nearest = msg->balls[0];
      tx_msg_.ball_camera.center_x = nearest.center_x;
      tx_msg_.ball_camera.center_y = nearest.center_y;
      tx_msg_.ball_camera.color = nearest.color;
      tx_msg_.ball_camera.confidence = nearest.confidence;
   } else {
      tx_msg_.ball_camera.center_x = 0;
      tx_msg_.ball_camera.center_y = 0;
      tx_msg_.ball_camera.color = 0;
      tx_msg_.ball_camera.confidence = 0.0;
   }
}

void Bridge::timerCallback() {
   MsgPacketizer::update();

   tx_msg_.ball_camera.center_x = 0;
   tx_msg_.ball_camera.center_y = 0;
   tx_msg_.ball_camera.color = 0;
   tx_msg_.ball_camera.confidence = 0.0;

   auto lidar_condition_msg = std_msgs::msg::Bool();
   lidar_condition_msg.data = rx_msg_.lidar_condition;
   lidar_condition_publisher_->publish(lidar_condition_msg);
   auto camera_condition_msg = std_msgs::msg::Bool();
   camera_condition_msg.data = rx_msg_.camera_condition;
   camera_condition_publisher_->publish(camera_condition_msg);

   if (debug_) {
      RCLCPP_INFO(this->get_logger(), "[echo] nearest circle: (%.2f, %.2f, %.2f)", rx_msg_.nearest_circle.center_x, rx_msg_.nearest_circle.center_y, rx_msg_.nearest_circle.radius);
      RCLCPP_INFO(this->get_logger(), "[echo] walls: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)", rx_msg_.walls[0].direction, rx_msg_.walls[0].distance, rx_msg_.walls[1].direction, rx_msg_.walls[1].distance, rx_msg_.walls[2].direction, rx_msg_.walls[2].distance, rx_msg_.walls[3].direction, rx_msg_.walls[3].distance);
      RCLCPP_INFO(this->get_logger(), "[echo] ball camera: (%d, %d, %d, %.2f)", rx_msg_.ball_camera.center_x, rx_msg_.ball_camera.center_y, rx_msg_.ball_camera.color, rx_msg_.ball_camera.confidence);
      RCLCPP_INFO(this->get_logger(), "[echo] lidar condition: %d, camera condition: %d", rx_msg_.lidar_condition, rx_msg_.camera_condition);
   }
}

int main(int argc, char *argv[]) {
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Bridge>());
   rclcpp::shutdown();
   return 0;
}

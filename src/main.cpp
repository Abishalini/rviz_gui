#include <QApplication>

#include "myviz.h"

#ifdef ROS
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

int main(int argc, char *argv[])
{
  QApplication app( argc, argv );

  MyViz* myviz = new MyViz();
  myviz->show();

#ifdef ROS
  // Initialize ROS
  rclcpp::init(arg, argv);

  auto node = rclcpp::NOde::make_shared("rviz_gui");

  // ROS and application loop
  rclcpp:WallRate loop_rate(20);
  while (rclcpp::ok()) {
      app.processEvent();
      rclcpp::spin_some(node);
      loop_rate.sleep();
  }
#else
  return app.exec();
#endif
}
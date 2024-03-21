#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <geometry_msgs/geometry_msgs/msg/point.hpp>

class GazeboForworder : public rclcpp::Node
{
public:
    GazeboForworder();
    void gz_odometry_topic_callback(const gz::msgs::Odometry &_msg);
    std::string gz_odometry_topic;
    gz::transport::Node subscribeNode;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr gz_odometry_publisher_;

private:
};
GazeboForworder::GazeboForworder() : Node("gazebo_forworder")
{
    this->declare_parameter("gz_odometry_topic", "/model/x500_1/odometry");
    gz_odometry_topic = this->get_parameter("gz_odometry_topic").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "gz_odometry_topic: %s", gz_odometry_topic.c_str());

    if (!subscribeNode.Subscribe(gz_odometry_topic, &GazeboForworder::gz_odometry_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", gz_odometry_topic.c_str());
    }
    gz_odometry_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(gz_odometry_topic, 10);
}
void GazeboForworder::gz_odometry_topic_callback(const gz::msgs::Odometry &_msg)
{
    geometry_msgs::msg::Point p;
    p.x = _msg.pose().position().x();
    p.y = _msg.pose().position().y();
    p.z = _msg.pose().position().z();
    gz_odometry_publisher_->publish(p);
}
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboForworder>());
    rclcpp::shutdown();

    return 0;
}

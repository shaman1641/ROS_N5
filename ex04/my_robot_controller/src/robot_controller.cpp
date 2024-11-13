#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>

class CircleMover : public rclcpp::Node
{
public:
    CircleMover() : Node("circle_mover")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CircleMover::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        publish_tf();
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Установите линейную скорость
        msg.angular.z = 0.5; // Установите угловую скорость
        publisher_->publish(msg);
    }

    void publish_tf()
    {
        // Публикация трансформации для робота
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "robot";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        // Создание кватерниона для начальной ориентации
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = 1.0; // Начальная ориентация (без поворота)
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = 0.0;

        transformStamped.transform.rotation = quaternion;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleMover>());
    rclcpp::shutdown();
    return 0;
}

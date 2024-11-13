#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <cmath>

class ButterflyMover : public rclcpp::Node
{
public:
    ButterflyMover() : Node("butterfly_mover"), t_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ButterflyMover::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        publish_tf();
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        
        // Calculate the velocities for the butterfly pattern
        msg.linear.x = 0.5 * std::sin(t_);  // Linear speed varies with time
        msg.angular.z = 0.5 * std::cos(t_); // Angular speed varies with time
        
        publisher_->publish(msg);
        
        // Update time for the next callback
        t_ += 0.1; // Increment time step
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
    double t_; // Time variable for movement pattern
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButterflyMover>());
    rclcpp::shutdown();
    return 0;
}

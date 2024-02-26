#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

double ar_x,ar_y,ar_z;
geometry_msgs::msg::Twist vel;
int marker_ID;


class MoveAr : public rclcpp::Node
{
    public: MoveAr()
    : Node("MoveToAr")
        {
        sub_ar = this->create_subscription<geometry_msgs::msg::PoseStamped>
        ("/aruco_single/pose",10,std::bind(&MoveAr::ARCB,this,_1)); 

        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

        }

    private:
        void ARCB(const geometry_msgs::msg::PoseStamped &msg)
        {
            std::cout<<"get"<<std::endl;
                ar_x = msg.pose.position.x;
                ar_y = msg.pose.position.y;
                ar_z = msg.pose.position.z;

                vel.linear.x =  0.5 * ar_x; 
                vel.angular.z = 1.0 * ar_y;
                pub_vel->publish(vel);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel.linear.x:%f;vel.angular.z:%f"
                        ,vel.linear.x,vel.angular.z);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ar;
        
};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    //rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_ar");

    rclcpp::spin(std::make_shared<MoveAr>());
    rclcpp::shutdown();
    return 0;
}


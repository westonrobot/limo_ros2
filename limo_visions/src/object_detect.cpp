#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/image.h>
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

using std::placeholders::_1;

geometry_msgs::msg::Twist twist_vel;
bool getimg = false;
double twist_linear_x = 0.0;
double twist_angular_z = 0.0;
sensor_msgs::msg::Image hsv_image;

class FollowLine : public rclcpp::Node
{
    public: FollowLine()
    : Node("FollowLine")
        {
        sub_img = image_transport::create_subscription(this,"/camera/color/image_raw",
                std::bind(&FollowLine::image_callback,this,_1),
                "raw",rmw_qos_profile_sensor_data);
        pub_img = image_transport::create_publisher(this,"/image_hsv",rmw_qos_profile_sensor_data);
        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

        }

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            cv_bridge::CvImagePtr cvImage;
            try
            {
                cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),"cv_bridge Exception:"<<e.what());
            }
            cv::Mat image = cvImage -> image;
            cv::Mat hsv = image.clone();
            cv::Mat res = image.clone();
            cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
            cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,46),res);
            int h = image.rows;
            int w = image.cols;
            int search_top = 3 * h / 4, search_bot = search_top + 20;
            for (int i = 0; i < search_top; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    res.at<uchar>(i,j) = 0;
                }
                
            }
            for (int i = 0; i < search_bot; i++)
            {
                for (int j = 0; j < w; j++)
                {
                    res.at<uchar>(i,j) = 0;
                }
                
            }
            cv::Moments M = cv::moments(res);
            if (M.m00 > 0)
            {
                int cx = int (cvRound(M.m10/M.m00));
                int cy = int (cvRound(M.m01/M.m00));
                RCLCPP_INFO(this->get_logger(),"cx: %d,cy %d",cx,cy);
                cv::circle(image,cv::Point(cx,cy),10,(0,0,255));
                double v = cx - w/2;
                twist_linear_x = 0.1;
                twist_angular_z = -v / 300 * 0.4;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"can not find line");
            }
            sensor_msgs::msg::Image::ConstPtr hsv_image_ = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image).toImageMsg();
            hsv_image = *hsv_image_;

            twist_vel.linear.x = twist_linear_x;
            twist_vel.angular.z = twist_angular_z;
            pub_vel->publish(twist_vel);
            pub_img.publish(hsv_image);
        }   

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        image_transport::Publisher pub_img;
        image_transport::Subscriber sub_img;
};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    //rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_ar");
   

    rclcpp::spin(std::make_shared<FollowLine>());
    rclcpp::shutdown();
    return 0;
}


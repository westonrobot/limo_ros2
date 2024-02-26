#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/image.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using std::placeholders::_1;

geometry_msgs::msg::Twist twist_vel;
bool getimg = false;
double twist_linear_x = 0.0;
double twist_angular_z = 0.0;
sensor_msgs::msg::Image hsv_image;

class Detectline : public rclcpp::Node
{
    public: Detectline()
    : Node("Detectline")
        {
        sub_img = image_transport::create_subscription(this,"/camera/color/image_raw",
                std::bind(&Detectline::image_callback,this,_1),
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
            // cv::Mat hsv = image.clone();
            // cv::Mat res = image.clone();
            // cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
            
            cv::Mat hsv;
            // 将图像从 BGR 转换到 HSV 颜色空间
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

            cv::Scalar lower_yellow(5, 80, 100);
            cv::Scalar upper_yellow(18, 200, 255);

            cv::Mat mask;
            // cv::inRange(hsv,cv::Scalar(170,170,70),cv::Scalar(180,190,100),res);
            cv::inRange(hsv, lower_yellow, upper_yellow, mask);
            
            cv::Mat color_x = mask(cv::Rect(0, 400, 400, 1));
            cv::Mat color_y = mask(cv::Rect(300, 400, 40, 80));
            cv::Mat color_sum = mask;

            int white_count_x = cv::countNonZero(color_x);
            int white_count_y = cv::countNonZero(color_y);
            int white_count_sum = cv::countNonZero(color_sum);

            // 查找白色像素点的位置
            std::vector<cv::Point> white_index_x, white_index_y;
            cv::findNonZero(color_x, white_index_x);
            cv::findNonZero(color_y, white_index_y);
          
        // 计算道路线中心位置
        int center_x, center_y;
        if (white_count_y == 0)
        {
            center_y = 240;
        }
        else
        {
            center_y = (white_index_y[white_count_y - 2].y + white_index_y[0].y) / 2;
            center_y += 340;
        }

        if (white_count_x == 0)
        {
            center_x = 195;
        }
        else
        {
            center_x = (white_index_x[white_count_x - 2].x + white_index_x[0].x) / 2;
        }

        // 创建一个 Pose 消息，表示道路线的位置
        geometry_msgs::msg::Pose objPose;
        objPose.position.x = center_x;
        objPose.position.y = center_y;
        objPose.position.z = white_count_sum;

        follow(objPose);

        sensor_msgs::msg::Image::ConstPtr hsv_image_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        hsv_image = *hsv_image_;

        // pub_vel->publish(twist_vel);
        pub_img.publish(hsv_image);
        }   

    void follow(geometry_msgs::msg::Pose msg)
    {
        geometry_msgs::msg::Twist vel;

        double x = msg.position.x;
        double y = msg.position.y;
        double z = msg.position.z;

        // 设置速度范围
        const double max_ang_vel = 0.8;
        const double min_ang_vel = -0.8;

        // 当检测不到道路线时，停止
        if (z <= 5)
        {
            vel.linear.x = 0;
            vel.angular.z = 0;
            pub_vel->publish(vel);
            RCLCPP_INFO(this->get_logger(), "Publish velocity command [%f m/s, %f rad/s]", vel.linear.x, vel.angular.z);
            return;
        }

        double lin_vel, ang_vel;
        // 检测一条道路线时，道路中心在此范围直行
        if (x < 210 && x > 190)
        {
            lin_vel = 0.19;
            ang_vel = 0;
        }
        // 偏差太大时，原地转向
        else if (x > 500 || x < 50)
        {
            lin_vel = 0;
            ang_vel = (1 - x / 210);
        }
        // 偏差较小时微调车身位置
        else
        {
            lin_vel = 0.19;
            ang_vel = (1 - x / 200) * 0.7;
        }

        // 转弯
        if (x == 195 && y >= 350)
        {
            lin_vel = 0.19;
            ang_vel = -0.60;
        }

        // 设定转向速度范围
        if (ang_vel >= max_ang_vel)
        {
            ang_vel = max_ang_vel;
        }
        if (ang_vel <= min_ang_vel)
        {
            ang_vel = min_ang_vel;
        }

        // 发布速度指令
        vel.linear.x = lin_vel;
        vel.angular.z = ang_vel;
        pub_vel->publish(vel);
        RCLCPP_INFO(this->get_logger(), "Publish velocity command [%f m/s, %f rad/s]", vel.linear.x, vel.angular.z);
    }



        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
        image_transport::Publisher pub_img;
        image_transport::Subscriber sub_img;
};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    //rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_ar");
   

    rclcpp::spin(std::make_shared<Detectline>());
    rclcpp::shutdown();
    return 0;
}


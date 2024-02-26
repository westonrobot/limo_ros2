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

using std::placeholders::_1;

geometry_msgs::msg::Twist twist_vel;
bool getimg = false;
double twist_linear_x = 0.0;
double twist_angular_z = 0.0;
sensor_msgs::msg::Image detect_image;

class Detectline : public rclcpp::Node
{
    public: Detectline()
    : Node("Detectline")
        {
        sub_img = image_transport::create_subscription(this,"/camera/color/image_raw",
                std::bind(&Detectline::image_callback,this,_1),
                "raw",rmw_qos_profile_sensor_data);
        pub_img = image_transport::create_publisher(this,"/image_hsv",rmw_qos_profile_sensor_data);

        }

    private:
      
void image_callback(const sensor_msgs::msg::Image::ConstPtr& image_msg)
{
    try
    {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

        // 在OpenCV中进行颜色识别
        cv::Mat image = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);  // 转换为HSV颜色空间

        // 定义颜色的阈值范围
        cv::Scalar lower_bound(30, 50, 50);  // 低阈值（颜色下限）
        cv::Scalar upper_bound(60, 255, 255);  // 高阈值（颜色上限）

        // 创建一个掩码，将在指定颜色范围内的像素设为白色，其他像素设为黑色
        cv::Mat mask;
        cv::inRange(hsv_image, lower_bound, upper_bound, mask);

        // 查找颜色区域的轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 在原图上绘制矩形框出颜色块
        for (const auto& contour : contours)
        {
            cv::Rect bounding_box = cv::boundingRect(contour);
            cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);  // 用红色矩形框出
        }

        // 在这里，你可以发布处理后的图像或执行其他操作

        // 显示图像
        // cv::imshow("Color Detection", image);
        // cv::waitKey(1);

        sensor_msgs::msg::Image::ConstPtr hsv_image_ = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image).toImageMsg();
        detect_image = *hsv_image_;
        pub_img.publish(detect_image);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),"cv_bridge Exception:"<<e.what());
    }
}

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


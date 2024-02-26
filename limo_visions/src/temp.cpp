void imageCB(const sensor_msgs::msg::Image &msg)
{
    cv_bridge::CvImageConstPtr cvImage;
    try
    {   //使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    cv::Mat image = cvImage->image;
    //深度拷贝
    cv::Mat hsv = image.clone();
    cv::Mat mask1 = image.clone();
    cv::Mat mask2 = image.clone();
    cv::Mat mask3 = image.clone();
    cv::Mat maskr = image.clone();
    cv::Mat maskg1 = image.clone();
    cv::Mat maskg2 = image.clone();
    cv::Mat maskg = image.clone();
    cv::Mat masky = image.clone();
    cv::Mat circles_r = image.clone();
    cv::Mat circles_y = image.clone();
    cv::Mat circles_g = image.clone();

    //将彩色图装换成灰度图
    cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
    //绘制文字
    cv::HersheyFonts front = cv::FONT_HERSHEY_SIMPLEX;
    cv::Mat img = hsv;
    cv::Mat cimg = img;
    //根据阈值去除背景，mask1为输出图像
    cv::inRange(hsv,cv::Scalar(0, 50, 220),cv::Scalar(30, 150, 255),mask1);
    cv::inRange(hsv,cv::Scalar(160, 100, 210),cv::Scalar(180, 245, 255),mask3);
    cv::inRange(hsv,cv::Scalar(40, 200, 150),cv::Scalar(90, 255, 255),maskg1);
    cv::inRange(hsv,cv::Scalar(5, 5, 234),cv::Scalar(35, 10, 235),maskg2);
    cv::inRange(hsv,cv::Scalar(15, 80, 180),cv::Scalar(35, 255, 255),masky);

    //融合两个区间的图像
    cv::add(mask1,mask3,maskr);
    cv::add(maskg1,maskg2,maskg);

    //腐蚀图像，进行闭运算
    cv::Mat kernel = cv::getStructuringElement(0, cv::Size(5, 5));
    cv::morphologyEx(maskr,maskr,3,kernel);
    //滤波
    cv::medianBlur(maskr,maskr,5);
    cv::medianBlur(masky,masky,5);
    cv::medianBlur(maskg,maskg,5);

    int h = img.rows;
    int w = img.cols;
    //霍夫圆检测，
    cv::HoughCircles(maskr , circles_r, cv::HOUGH_GRADIENT, 1, 80, 50, 10, 0, 20);
    cv::HoughCircles(masky, circles_y, cv::HOUGH_GRADIENT, 1, 60, 50, 10, 0, 20);
    cv::HoughCircles(maskg , circles_g, cv::HOUGH_GRADIENT, 1, 30, 50, 7, 0, 20);

    // cv::imshow("circles_r",circles_r);
    // cv::waitKey(1);
    // ROS_INFO("done circles_r");

    cv::Moments M_r = cv::moments(maskr);
    cv::Moments M_g = cv::moments(maskg);
    cv::Moments M_y = cv::moments(masky);

    int r = 5;
    double bound = 5.0/10;
    if (M_r.m00>0)
    {
        int cx = int (cvRound(M_r.m10/M_r.m00));
        int cy = int (cvRound(M_r.m01/M_r.m00));

        cv::circle(maskr,cv::Point(cx, cy), 10, CV_RGB(255,0,0), 2);
        cv::circle(cimg,cv::Point(cx, cy), 10, CV_RGB(255,0,0),2);
        cv::putText(cimg,"RED",cv::Point(cx, cy),front,2,CV_RGB(255,0,0),3);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"That is RED" );
    }

    if (M_g.m00>0)
    {
        int cx = int (cvRound(M_g.m10/M_g.m00));
        int cy = int (cvRound(M_g.m01/M_g.m00));

        cv::circle(maskg,cv::Point(cx, cy), 10, CV_RGB(0, 255, 0),2);
        cv::circle(cimg,cv::Point(cx, cy), 10, CV_RGB(0, 255, 0),2);
        cv::putText(cimg,"GREEN",cv::Point(cx, cy),front,2,CV_RGB(0, 255, 0),3);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"That is GREEN" );
    }

    if (M_y.m00>0)
    {
        int cx = int (cvRound(M_y.m10/M_y.m00));
        int cy = int (cvRound(M_y.m01/M_y.m00));

        cv::circle(masky,cv::Point(cx, cy), 10, CV_RGB(255, 255, 0),2);
        cv::circle(cimg,cv::Point(cx, cy), 10, CV_RGB(255, 255, 0),2);
        cv::putText(cimg,"YELLOW",cv::Point(cx, cy),front,2,CV_RGB(255,255,0),3);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"That is YELLOW" );
    }

    cv::cvtColor(cimg,cimg,cv::COLOR_HSV2BGR);
    sensor_msgs::ImagePtr hsv_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cimg).toImageMsg();
    hsv_image = *hsv_image_;

}

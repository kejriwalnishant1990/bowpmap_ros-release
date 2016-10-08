#include<iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"sendKinectImage");

    ros::NodeHandle nImage;

    ros::Publisher publishKinectImage;

    publishKinectImage = nImage.advertise<sensor_msgs::Image>("/camera/rgb/image_color",1);
    publishKinectImage.publish(sensor_msgs::Image());

    vector<cv::Mat> image(10);

    image[0] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0001.jpg").clone();
    image[1] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0002.jpg").clone();
    image[2] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0003.jpg").clone();
    image[3] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0004.jpg").clone();
    image[4] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0005.jpg").clone();
    image[5] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0006.jpg").clone();
    image[6] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0007.jpg").clone();
    image[7] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0008.jpg").clone();
    image[8] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0009.jpg").clone();
    image[9] = cv::imread("/home/nishant/nishant_workspace/bowpMap/bin/Output_CC/Images/0010.jpg").clone();

    int i=0;

    while(1)
    {
        if(i==10)
            i=0;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                       "bgr8", image[i]).toImageMsg();
        publishKinectImage.publish(msg);
        i++;

    }


    return 0;
}

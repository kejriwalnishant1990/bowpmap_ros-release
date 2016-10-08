#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <bowpmap_ros/incrementalTopoMapFinal.h>
#include <bowpmap_ros/bowpMapping_ros.h>

using namespace std;

/*
 * Ctrl+C signal catch function
*/
void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    sleep(5);

    exit(signum);
}

/*
 * Main Function
 */

int main(int argc, char **argv)
{
    // Initialsing ros node
    ros::init(argc,argv,"bowpmap_ros");


    BowpMap *bowpMap = new BowpMap();

    // Subsciber for camera image through kinect
    ros::Subscriber subImage = bowpMap->nImage.subscribe<sensor_msgs::Image>("/rgb/image",5,
                                                                    &BowpMap::loopClosureCallBack,bowpMap);

    // Subscriber for odometry reading
    ros::Subscriber subOdometry = bowpMap->nImage.subscribe<nav_msgs::Odometry>("/odom",1,&BowpMap::odometryCallback,bowpMap);

    // Initialising loop closure detection topic publisher
    bowpMap->loopClosureEventPublisher = bowpMap->nImage.advertise<std_msgs::Bool>("/loopClosure",1);
    bowpMap->loopClosureEventPublisher.publish(std_msgs::Bool());

    bowpMap->loopClosureImagePublisher = bowpMap->nImage.advertise<sensor_msgs::Image>("/loopClosureImage",1);
    bowpMap->loopClosureImagePublisher.publish(sensor_msgs::Image());

//    ros::MultiThreadedSpinner spinner(2);
//    spinner.spin();
    ros::spin();

    return 0;
}

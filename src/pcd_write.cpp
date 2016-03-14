#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

pcl::PointCloud<pcl::PointXYZRGB> cloud;
pcl::PCDWriter writer;
pcl::visualization::CloudViewer viewer("Kinect2");

const std::string imageWindowName = "Recording";

const float SCALE = 0.5;
const cv::Size size(1920 * SCALE, 1080 * SCALE);

void savePCD(bool ASCII)
{
    std::string file;
    if (ASCII)
    {
        file = "data/test_pcd_ascii_" + boost::lexical_cast<std::string>(ros::Time::now().toNSec()) + ".pcd";
        writer.writeASCII(file, cloud);
    }
    else
    {
        file = "data/test_pcd_" + boost::lexical_cast<std::string>(ros::Time::now().toNSec()) + ".pcd";
        writer.writeBinary(file, cloud);
    }
    std::cout << "Saved " << cloud.size() << " data points to " << file << "." << std::endl;
}

// sensor_msgs/PointCloud2
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
    // ROS_INFO("Receiving points");

    pcl::fromROSMsg(*points, cloud);

    viewer.showCloud(cloud.makeShared());
}

void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::resize(cv_ptr->image, cv_ptr->image, size);
      cv::imshow(imageWindowName, cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_write");

    ros::NodeHandle n;

    writer = pcl::PCDWriter();

    // Try to view the cloud
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1000, pointsCallback); // sd or hd
    ros::Subscriber subImg = n.subscribe<sensor_msgs::Image>("/kinect2/hd/image_color_rect", 1000, imageCallback);

    cv::Mat empty = cv::Mat::zeros(500,500,CV_64F);
    cv::imshow(imageWindowName,empty);

    bool quit = false;
    do
    {
        ros::spinOnce();

        int key = cv::waitKey(10);

        bool save = false;
        bool saveASCII = false;
        switch(key & 0xFF)
        {
          case ' ':
          case 's':
            save = true;
            break;
          case 'x':
            saveASCII = true;
            break;
          case 'q':
            quit = true;
        }

        if (save)
        {
            savePCD(false);
        }
        else { }
        if (saveASCII)
        {
            savePCD(true);
        }
    } while (!quit && !viewer.wasStopped());

    return 0;
}

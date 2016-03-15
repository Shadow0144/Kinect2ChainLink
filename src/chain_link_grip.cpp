/*
 * ChainLink.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: shadow0144
 */

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
#include <ros/package.h>
#include <chain_link_grip.h>
#include <chain_link_alignment.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <pcl/filters/conditional_removal.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <boost/bind.hpp>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/Empty.h>
//#include <chain_link/gripRequest.h>
//#include <chain_link/gripResponse.h>
#include "chain_link/grip.h"
#include <chain_link_rotations.h>

bool view;
bool kinect;
bool scene;
bool debug;
pcl::PointCloud<pcl::PointXYZRGB> previousCloud;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
pcl::PointCloud<pcl::PointXYZRGB> nextCloud;
pcl::PointCloud<pcl::PointXYZRGB> gripCloud;
pcl::PointCloud<pcl::PointXYZRGB> debugCloud;
bool cloudSet = false;
TemplateAligner aligner;
pcl::visualization::PCLVisualizer* viewer;
pcl::visualization::PCLVisualizer* linkViewer;
pcl::visualization::PCLVisualizer* sceneViewer;
pcl::visualization::PCLVisualizer* debugViewer;
const std::string imageWindowName = "Kinect2Img";
const float SCALE = 0.5;
const cv::Size size(1920 * SCALE, 1080 * SCALE);
bool paused = false;
bool cloudAvailable = false;
int linkViewed = 0;
const std::string linkText = "linkText";
const std::string linkCloud = "linkCloud";
const std::string targetCloudName = "targetCloud";
const std::string templateCloudName = "templateCloud";
const std::string sceneCloudName = "sceneCloud";
const std::string sceneCloudMatchName = "sceneCloudMatch";
const std::string gripCloudName = "gripCloud";
const std::string debugCloudName = "debugCloud";
float x_range = 0.15f; // (Half the range)
float y_range = 0.15f; // (Half the range)
float z_range = 0.75f; // (Half the range)
int rMax = 50;
int gMax = 50;
int bMax = 50;
int align_max_iterations;
pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
const int pointSize = 3;
const int boldPointSize = 5;
tf::TransformListener* listener;
bool quit;
bool pauseOption;
int p; // Auto-pause parameters
int q = 1;

void SetupParamServer(ros::NodeHandle n)
{
    // Set values if they don't already exist so they are advertized
    if (!n.hasParam(target_voxel_grid_size_param))
        n.setParam(target_voxel_grid_size_param, aligner.GetTargetVoxelGridSize());
    if (!n.hasParam(target_min_x_param))
        n.setParam(target_min_x_param, aligner.GetxMinTarget());
    if (!n.hasParam(target_max_x_param))
        n.setParam(target_max_x_param, aligner.GetxMaxTarget());
    if (!n.hasParam(target_min_y_param))
        n.setParam(target_min_y_param, aligner.GetyMinTarget());
    if (!n.hasParam(target_max_y_param))
        n.setParam(target_max_y_param, aligner.GetyMaxTarget());
    if (!n.hasParam(target_min_z_param))
        n.setParam(target_min_z_param, aligner.GetzMinTarget());
    if (!n.hasParam(target_max_z_param))
        n.setParam(target_max_z_param, aligner.GetzMaxTarget());
    if (!n.hasParam(template_voxel_grid_size_param))
        n.setParam(template_voxel_grid_size_param, aligner.GetTemplateVoxelGridSize());
    if (!n.hasParam(target_x_range_param))
        n.setParam(target_x_range_param, x_range * 2);
    if (!n.hasParam(target_y_range_param))
        n.setParam(target_y_range_param, y_range * 2);
    if (!n.hasParam(target_z_range_param))
        n.setParam(target_z_range_param, z_range * 2);
    if (!n.hasParam(target_max_r_param))
        n.setParam(target_max_r_param, rMax);
    if (!n.hasParam(target_max_g_param))
        n.setParam(target_max_g_param, gMax);
    if (!n.hasParam(target_max_b_param))
        n.setParam(target_max_b_param, bMax);
    if (!n.hasParam(align_max_iterations_param))
        n.setParam(align_max_iterations_param, aligner.GetMaxIterations());
}

void FilterCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = cloud.makeShared();

    int s = cloud.size();
    printf("Initially filter cloud of size %d points... ", s);

    filter.setInputCloud(cloudPtr);
    filter.filter(cloud);

    printf("Removed %d yellow points.\n", s - cloud.size());

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloudPtr);

    pcl::PointXYZRGB searchPoint;
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;

    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    float x = cloud.points[ pointIdxNKNSearch[0] ].x;
    float y = cloud.points[ pointIdxNKNSearch[0] ].y;
    float z = cloud.points[ pointIdxNKNSearch[0] ].z;
    aligner.SetTargetWindow(
                x - x_range, x + x_range,
                y - y_range, y + y_range,
                z - z_range, z + z_range);
}

void HandleParams(ros::NodeHandle n)
{
    float targetVoxelSize, templateVoxelSize,
            xMinTarget, /*xMinTemplate,*/
            xMaxTarget, /*xMaxTemplate,*/
            yMinTarget, /*yMinTemplate,*/
            yMaxTarget, /*yMaxTemplate,*/
            zMinTarget, /*zMinTemplate,*/
            zMaxTarget/*, zMaxTemplate*/;

    if (n.getParam(target_voxel_grid_size_param, targetVoxelSize))
    {
        if (targetVoxelSize != aligner.GetTargetVoxelGridSize())
        {
            aligner.SetTargetVoxelGridSize((targetVoxelSize));
        }
        else { }
    }
    else { }

    if (n.getParam(template_voxel_grid_size_param, templateVoxelSize))
    {
        if (templateVoxelSize != aligner.GetTemplateVoxelGridSize())
        {
            aligner.SetTemplateVoxelGridSize(templateVoxelSize);
        }
        else { }
    }
    else { }

    if (    n.getParam(target_min_x_param, xMinTarget) &&
            n.getParam(target_max_x_param, xMaxTarget) &&
            n.getParam(target_min_y_param, yMinTarget) &&
            n.getParam(target_max_y_param, yMaxTarget) &&
            n.getParam(target_min_z_param, zMinTarget) &&
            n.getParam(target_max_z_param, zMaxTarget)  )
    {
        aligner.SetTargetWindow(xMinTarget, xMaxTarget,
                                   yMinTarget, yMaxTarget,
                                   zMinTarget, zMaxTarget);
    }
    else { }

    /*if (    n.getParam(template_min_x_param, xMinTemplate) &&
            n.getParam(template_max_x_param, xMaxTemplate) &&
            n.getParam(template_min_y_param, yMinTemplate) &&
            n.getParam(template_max_y_param, yMaxTemplate) &&
            n.getParam(template_min_z_param, zMinTemplate) &&
            n.getParam(template_max_z_param, zMaxTemplate)  )
    {
        aligner.ResizeTemplateWindow(xMinTemplate, xMaxTemplate,
                                   yMinTemplate, yMaxTemplate,
                                   zMinTemplate, zMaxTemplate);
    }
    else { }*/

    if (    n.getParam(target_x_range_param, x_range)   )
    {
        x_range /= 2;
    }
    else { }

    if (    n.getParam(target_y_range_param, y_range)   )
    {
        y_range /= 2;
    }
    else { }

    if (    n.getParam(target_z_range_param, z_range)   )
    {
        z_range /= 2;
    }
    else { }

    if (    n.getParam(target_max_r_param, rMax)   );
    if (    n.getParam(target_max_g_param, gMax)   );
    if (    n.getParam(target_max_b_param, bMax)   );

    int maxIter;
    if (n.getParam(align_max_iterations_param, maxIter))
    {
        if (maxIter != aligner.GetMaxIterations())
        {
            aligner.SetMaxIterations(maxIter);
        }
        else { }
    }
    else { }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> eviewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "p" && event.keyDown ())
  {
      paused = !(paused);
      if (paused) { printf("Paused\n"); }
      else { printf("Unpaused\n"); }
  }
  else if (event.getKeySym () == "q" && event.keyDown ())
  {
      quit = true;
  }
  else { }
}

bool unpressedLeft = true;
bool unpressedRight = true;
void linkKeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* link_viewer_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> eviewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown())
  {
      if (unpressedLeft && (event.getKeyCode() == 60 || event.getKeyCode() == 44)) // "<" || ","
      {
          linkViewed = (--linkViewed+aligner.GetNumTemplates())%aligner.GetNumTemplates();
          linkViewer->updateText("Link #" + boost::lexical_cast<std::string>(linkViewed), 10, 10, linkText);
          linkViewer->updatePointCloud(aligner.GetTemplate(linkViewed), linkCloud);
          unpressedLeft = false;
      }
      else if (unpressedRight && (event.getKeyCode() == 62 || event.getKeyCode() == 46)) // ">" || "."
      {
          linkViewed = (++linkViewed+aligner.GetNumTemplates())%aligner.GetNumTemplates();
          linkViewer->updateText("Link #" + boost::lexical_cast<std::string>(linkViewed), 10, 10, linkText);
          linkViewer->updatePointCloud(aligner.GetTemplate(linkViewed), linkCloud);
          unpressedRight = false;
      }
      else if (event.getKeySym () == "q")
      {
          quit = true;
      }
      else { }
  }
  else if (event.keyUp()) // Doesn't actually work
  {
      if (event.getKeyCode() == 60 || event.getKeyCode() == 44)
      {
          unpressedLeft = true;
      }
      if (event.getKeyCode() == 62 || event.getKeyCode() == 46)
      {
          unpressedRight = true;
      }
      else { }
  }
  else { }
}

bool addOnce = false;
// sensor_msgs/PointCloud2
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
    if (!paused || !cloudAvailable)
    {
        pcl::fromROSMsg(*points, nextCloud);
        if (cloudSet)
        {
            cloud = previousCloud + nextCloud;
        }
        else
        {
            cloud = nextCloud;
            //cloudSet = true;
        }
        previousCloud = nextCloud;

        ROS_INFO("Cloud received");
        FilterCloud();
        aligner.SetTargetCloud(cloud.makeShared());

        aligner.Align();

        cloudAvailable = true;

        if (view)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> targetColor
                (aligner.GetTargetCloud(), 0, 255, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> templateColor
                (aligner.GetMostAlignedTemplate(), 0, 255, 0);
            if (!addOnce)
            {
                viewer->addPointCloud(aligner.GetTargetCloud(), targetColor, targetCloudName);
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, targetCloudName);
                viewer->addPointCloud(aligner.GetMostAlignedTemplate(), templateColor, templateCloudName);
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, templateCloudName);
                addOnce = true;
            }
            else
            {
                viewer->updatePointCloud(aligner.GetTargetCloud(), targetColor, targetCloudName);
                viewer->updatePointCloud(aligner.GetMostAlignedTemplate(), templateColor, templateCloudName);
            }
        }
        else { }

        q++;
        if (q%p == 0 && pauseOption)
        {
            printf("Paused\n");
            paused = true;
        }
        else { }
    }
    else { }
}

bool addSceneCloudOnce = false;
// sensor_msgs/PointCloud2
void sceneCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
    if (cloudAvailable && !paused)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> templateColor
            (aligner.GetMostAlignedTemplate(), 0, 255, 0);
        pcl::PointCloud<pcl::PointXYZRGB> sceneCloud;

        ROS_INFO("Scene Received");

        try
        {
            if (!kinect)
            {
                sensor_msgs::PointCloud2 transformed;

                pcl_ros::transformPointCloud("/container", *points, transformed, *listener);

                pcl::fromROSMsg(transformed, sceneCloud);
            }
            else
            {
                pcl::fromROSMsg(*points, sceneCloud);
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            pcl::fromROSMsg(*points, sceneCloud);
        }

        if (!addSceneCloudOnce)
        {
            sceneViewer->addPointCloud(sceneCloud.makeShared(), sceneCloudName);
            sceneViewer->addPointCloud(aligner.GetMostAlignedTemplate(), templateColor, sceneCloudMatchName);
            sceneViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, sceneCloudMatchName);
            addSceneCloudOnce = true;
        }
        else
        {
            sceneViewer->updatePointCloud(sceneCloud.makeShared(), sceneCloudName);
            sceneViewer->updatePointCloud(aligner.GetMostAlignedTemplate(), templateColor, sceneCloudMatchName);
        }
    }
    else { }
}

bool axesAdded = false;
bool gripPointsCallback(chain_link::gripRequest& request, chain_link::gripResponse& response)
{
    ROS_INFO("Requested grip");

    response.header = std_msgs::Header();
    response.header.stamp = ros::Time::now();

    tf::StampedTransform transform;

    if (!cloudAvailable)
    {
        return -1;
    }
    else { }

    try
    {
        if (!kinect)
        {
            listener->lookupTransform("/kinect2_link", "/container",
                                   ros::Time(0), transform);
        }
        else
        {
            listener->lookupTransform("/kinect2_link", "/kinect2_link",
                                   ros::Time(0), transform);
        }

        // Apply the inverse rotation from the header
        Eigen::Vector3f t (
            transform.getOrigin().getX(),
            transform.getOrigin().getY(),
            transform.getOrigin().getZ());
        Eigen::Quaternionf r (
            transform.getRotation().getW(),
            transform.getRotation().getX(),
            transform.getRotation().getY(),
            transform.getRotation().getZ());

        //Eigen::Quaternionf q (Views[aligner.GetMostAlignedTemplateIndex()]);
        // This could be made much more efficient by multiplying the rotations together



        // Apply to grip points
        pcl::PointCloud<pcl::PointXYZRGB> transformedGripCloud;
        pcl::transformPointCloud(gripCloud, transformedGripCloud, t, r);

        // Select one grip point (the one with the lowest z should always be the best)
        int index = 0;
        int minZ = transformedGripCloud.points[index].z;
        for (int i = 1; i < transformedGripCloud.points.size(); i++)
        {
            if (transformedGripCloud.points[i].z < minZ)
            {
                minZ = transformedGripCloud.points[i].z;
                index = i;
            }
            else { }
        }

        // Put the data into the response
        response.pose.position.x = transformedGripCloud.points[index].x;
        response.pose.position.y = transformedGripCloud.points[index].y;
        response.pose.position.z = transformedGripCloud.points[index].z;

// Fix rotation
        response.pose.orientation.x = transform.getRotation().x();
        response.pose.orientation.y = transform.getRotation().y();
        response.pose.orientation.z = transform.getRotation().z();
        response.pose.orientation.w = transform.getRotation().w();

        ROS_INFO("Grip sent:");
        ROS_INFO("Position x: %f, y: %f, z: %f",
                 response.pose.position.x,
                 response.pose.position.y,
                 response.pose.position.z);
        ROS_INFO("Rotation x: %f, y: %f, z: %f, w: %f",
                 response.pose.orientation.x,
                 response.pose.orientation.y,
                 response.pose.orientation.z,
                 response.pose.orientation.w);

        if (debug)
        {
            pcl::PointCloud<pcl::PointXYZRGB> axes;
            axes.width    = 4;
            axes.height   = 1;
            axes.is_dense = false;
            axes.points.resize (axes.width * axes.height);

            axes.points[0].x = 0.0f;
            axes.points[0].y = 0.0f;
            axes.points[0].z = 0.0f;

            axes.points[1].x = 0.1f;
            axes.points[1].y = 0.0f;
            axes.points[1].z = 0.0f;

            axes.points[2].x = 0.0f;
            axes.points[2].y = 0.1f;
            axes.points[2].z = 0.0f;

            axes.points[3].x = 0.0f;
            axes.points[3].y = 0.0f;
            axes.points[3].z = 0.1f;

            pcl::transformPointCloud(axes, axes, t, r);

            pcl::PointCloud<pcl::PointXYZRGB> rotCloud;
            pcl::transformPointCloud(debugCloud, rotCloud, t, r);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> debugCloudColor
                (aligner.GetMostAlignedTemplate(), 255, 255, 255);
            debugViewer->updatePointCloud(rotCloud.makeShared(), debugCloudColor, debugCloudName);

            debugViewer->removeAllShapes();
            debugViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[1], 255, 0, 0, "x");
            debugViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[2], 0, 255, 0, "y");
            debugViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[3], 0, 0, 255, "z");

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgcShared = transformedGripCloud.makeShared();

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
                (transformedGripCloud.makeShared(), 255, 0, 0);
            debugViewer->updatePointCloud(tgcShared, gripColor, gripCloudName);

            if (scene) // Add it to the scene too if both are enabled
            {
                sceneViewer->updatePointCloud(rotCloud.makeShared(), debugCloudColor, debugCloudName);
                sceneViewer->removeAllShapes();
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[1], 150, 0, 0, "x");
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[2], 0, 150, 0, "y");
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[3], 0, 0, 150, "z");

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
                    (tgcShared, 255, 0, 0);
                sceneViewer->updatePointCloud(tgcShared, gripColor, gripCloudName);
            }
            else { }

            axesAdded = true;
        }
        else { }
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    return true;
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

int main(int argc, char** argv)
{
    quit = false;
    kinect = false;
    view = false;
    debug = false;
    scene = false;
    bool link = false;
    bool image = false;
    std::string topic = "/rectangle_detection_node/box_content";
    std::string dpi = "hd";
    std::string serviceName = "chain_link_grip";
    char* templates;

    printf("Starting...\n");

    // Check if we have enough parameters
    if (argc < 2)
    {
        printf ("No templates file given!\n");
        printf ("Use ./chain_link_grip <templates.txt> [-a] [-k [<sd or qhd or hd>] or -t <topic>] [-l] [-v] [-i] [-p <interval>]\n");
        return (-1);
    }

    // Make sure our templates file exists
    templates = argv[1];
    struct stat buffer;
    if (stat (templates, &buffer) != 0)
    {
        printf("File %s not found\n", templates);
        return (-1);
    }
    else { }

    // Addtional parameters if any
    if (argc > 2)
    {
        if (strcmp(argv[2], "-h") == 0)
        {
            printf(""); // TODO
            return 0;
        }
        else { }

        for (int i = 2; i < argc; i++)
        {
            /* Various input types */
            if (strcmp(argv[i], "-k") == 0)
            {
                kinect = true;
                if (argc > i+1)
                {
                    dpi = argv[++i];
                }
                else { }
                topic = "/kinect2/" + dpi + "/points";
            }
            else if (strcmp(argv[i], "-t") == 0)
            {
                kinect = false;
                if (argc > i+1)
                {
                    topic = argv[++i];
                }
                else
                {
                    printf ("No topic given!\n");
                    return (-1);
                }
            }
            /* /Various input types */
            /* Display options */
            else if (strcmp(argv[i], "-a") == 0)
            {
                view = true;
            }
            else if (strcmp(argv[i], "-l") == 0)
            {
                link = true;
            }
            else if (strcmp(argv[i], "-v") == 0)
            {
                scene = true;
            }
            else if (strcmp(argv[i], "-i") == 0)
            {
                image = true;
            }
            else if (strcmp(argv[i], "-d") == 0)
            {
                debug = true;
            }
            /* /Display options */
            /* Other */
            else if (strcmp(argv[i], "-p") == 0)
            {
                try
                {
                    if (argc > i+1)
                    {
                        p = atoi(argv[++i]);
                        if (p < 1)
                        {
                            printf ("Inverval must be > 0!\n");
                            return (-1);
                        }
                        pauseOption = true;
                    }
                    else
                    {
                        printf ("No interval given!\n");
                        return (-1);
                    }
                }
                catch (std::exception)
                {
                    printf ("Interval must be an integer!\n");
                    return (-1);
                }
            }
            else { }
        }
    }
    else { }

    ros::init(argc, argv, "chain_link_grip");

    ros::NodeHandle n;

    printf("Initializing aligner...\n");

    aligner = TemplateAligner(argv[1]); // object templates, target

    // Set up parameter server
    SetupParamServer(n);

    listener = new tf::TransformListener();

    //Create filtering codition. Points less than the limits will remain.
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr condition
            (new pcl::ConditionAnd<pcl::PointXYZRGB>);
    condition->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr
                             (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                              ("r", pcl::ComparisonOps::LT, rMax)));
    condition->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr
                             (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                              ("g", pcl::ComparisonOps::LT, gMax)));
    condition->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr
                             (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                              ("b", pcl::ComparisonOps::LT, bMax)));

    //Filter object
    filter.setCondition(condition);

    // Try to view the cloud
    ros::Subscriber sub;
    ros::Subscriber subImg;
    ros::Subscriber subScene;

    printf("Subscribing to topic %s...\n", topic.c_str());

    sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 10, pointsCallback);

    if (image)
    {
        subImg = n.subscribe<sensor_msgs::Image>("/kinect2/hd/image_color", 10, imageCallback);
    }
    else { }

    if (scene)
    {
        subScene = n.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 10, sceneCallback);
    }
    else { }

    if (view)
    {
        viewer = new pcl::visualization::PCLVisualizer ("Aligned");
        viewer->initCameraParameters ();
        viewer->addCoordinateSystem (0.1);
        viewer->setBackgroundColor (0.15, 0.15, 0.15, 0);
        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    }
    else { }

    // Set up the points and rotations of the views and grips
    InitPointsAndRotatations();

    // Create a grip cloud to view
    // Fill in the cloud data
    gripCloud.width    = grip_num;
    gripCloud.height   = 1;
    gripCloud.is_dense = false;
    gripCloud.points.resize (gripCloud.width * gripCloud.height);

    for (int i = 0; i < grip_num; i++)
    {
        gripCloud.points[i].x = GripPoints[i][0];
        gripCloud.points[i].y = GripPoints[i][1];
        gripCloud.points[i].z = GripPoints[i][2];
    }

    /*float offset = 0.0025f;
    for (int i = 8; i < gripCloud.points.size();)
    {
        int j = (i - 8) / 6;

        gripCloud.points[i].x = gripCloud.points[j].x + offset;
        gripCloud.points[i].y = gripCloud.points[j].y;
        gripCloud.points[i].z = gripCloud.points[j].z;
        i++;

        gripCloud.points[i].x = gripCloud.points[j].x - offset;
        gripCloud.points[i].y = gripCloud.points[j].y;
        gripCloud.points[i].z = gripCloud.points[j].z;
        i++;

        gripCloud.points[i].x = gripCloud.points[j].x;
        gripCloud.points[i].y = gripCloud.points[j].y + offset;
        gripCloud.points[i].z = gripCloud.points[j].z;
        i++;

        gripCloud.points[i].x = gripCloud.points[j].x;
        gripCloud.points[i].y = gripCloud.points[j].y - offset;
        gripCloud.points[i].z = gripCloud.points[j].z;
        i++;

        gripCloud.points[i].x = gripCloud.points[j].x;
        gripCloud.points[i].y = gripCloud.points[j].y;
        gripCloud.points[i].z = gripCloud.points[j].z + offset;
        i++;

        gripCloud.points[i].x = gripCloud.points[j].x;
        gripCloud.points[i].y = gripCloud.points[j].y;
        gripCloud.points[i].z = gripCloud.points[j].z - offset;
        i++;
    }*/

    if (link)
    {
        linkViewer = new pcl::visualization::PCLVisualizer ("Link Clouds");
        linkViewer->initCameraParameters ();
        linkViewer->addCoordinateSystem (0.1);
        linkViewer->setBackgroundColor (0.25, 0.25, 0.25, 0);
        linkViewer->addText ("Link #" + boost::lexical_cast<std::string>(linkViewed), 10, 10, linkText);
        linkViewer->addPointCloud(aligner.GetTemplate(linkViewed), linkCloud);
        linkViewer->registerKeyboardCallback (linkKeyboardEventOccurred, (void*)&linkViewer);


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
            (gripCloud.makeShared(), 255, 0, 0);
        linkViewer->addPointCloud(gripCloud.makeShared(), gripColor, gripCloudName);
        linkViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, gripCloudName);
    }
    else { }

    if (debug)
    {
        debugViewer = new pcl::visualization::PCLVisualizer ("Debug");
        debugViewer->initCameraParameters ();
        debugViewer->addCoordinateSystem (0.1);
        debugViewer->setBackgroundColor (0.0, 0.0, 0.0, 0);

        const std::string &pcd_file = "./data/ChainLinkResizeCenter.pcd";
        if (pcl::io::loadPCDFile (pcd_file, debugCloud) == -1)
        {
            printf ("Error loading file %s\n", pcd_file.c_str());
            return -1;
        }
        else { }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> debugCloudColor
            (aligner.GetMostAlignedTemplate(), 255, 255, 255);
        debugViewer->addPointCloud(debugCloud.makeShared(), debugCloudColor, debugCloudName);
        debugViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, debugCloudName);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
            (gripCloud.makeShared(), 255, 0, 0);
        debugViewer->addPointCloud(gripCloud.makeShared(), gripColor, gripCloudName);
        debugViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, boldPointSize, gripCloudName);
    }
    else { }

    if (scene)
    {
        sceneViewer = new pcl::visualization::PCLVisualizer ("Scene");
        sceneViewer->initCameraParameters ();
        sceneViewer->addCoordinateSystem (0.1);
        sceneViewer->setBackgroundColor (0.0, 0.0, 0.0, 0);

        // Add the matched template to the scene too
        if (debug)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> debugCloudColor
                (debugCloud.makeShared(), 255, 255, 255);
            sceneViewer->addPointCloud(debugCloud.makeShared(), debugCloudColor, debugCloudName);
            sceneViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, debugCloudName);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
                (gripCloud.makeShared(), 255, 0, 0);
            sceneViewer->addPointCloud(gripCloud.makeShared(), gripColor, gripCloudName);
            sceneViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, boldPointSize, gripCloudName);
        }
        else { }
    }
    else { }

    printf("Starting service %s...\n", serviceName.c_str());

    // Set up the service to provide grip points
    ros::ServiceServer service;
    service = n.advertiseService(serviceName, gripPointsCallback);

    printf("Running...\n");

    // Main loop
    do
    {
        HandleParams(n); // Check for updated parameters

        ros::spinOnce();

        if (view) viewer->spinOnce();
        if (link) linkViewer->spinOnce();
        if (scene) sceneViewer->spinOnce();
        if (debug) debugViewer->spinOnce();
    }
    while (ros::ok() && !quit);

    // Begin shutdown
    if (view) delete viewer;
    if (link) delete linkViewer;
    if (scene) delete sceneViewer;
    if (debug) delete debugViewer;

    delete listener;

    printf("Stopped\n");

    return 0;
}

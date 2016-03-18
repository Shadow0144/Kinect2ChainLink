/*
 * ChainLink.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: shadow0144
 */

//
// <Includes>
//

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
#include <tf_conversions/tf_eigen.h>
//#include <chain_link/gripRequest.h>
//#include <chain_link/gripResponse.h>
#include "chain_link/grip.h"
#include <chain_link_rotations.h>
#include <tf/transform_broadcaster.h>

//
// </Includes>
// <Parameters>
//

// Service and topic names
const std::string topicName = "chain_link_grip/pose";
const std::string chainLinkFrameId = "chain_link_grip_pose";
const std::string defaultTopic = "/rectangle_detection_node/box_content";
const std::string defaultDPI = "hd";

// View booleans
bool view;
bool kinect;
bool scene;
bool links;
bool image;

// Point Clouds
//pcl::PointCloud<pcl::PointXYZRGB> previousCloud;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
//pcl::PointCloud<pcl::PointXYZRGB> nextCloud;
pcl::PointCloud<pcl::PointXYZRGB> gripCloud;
pcl::PointCloud<pcl::PointXYZRGB> transformedGripCloud;
pcl::PointCloud<pcl::PointXYZRGB> fullLinkCloud;
pcl::PointCloud<pcl::PointXYZRGB> sceneCloud;

// Used if trying to combine the next and previous cloud (disabled)
//bool cloudSet;

// Aligner that tries to find templates in the scene
TemplateAligner aligner;

// The different viewers
pcl::visualization::PCLVisualizer* viewer;
pcl::visualization::PCLVisualizer* linkViewer;
pcl::visualization::PCLVisualizer* sceneViewer;

// Scale of the image view if it's enabled
const float SCALE = 0.5;
const cv::Size windowSize(1920 * SCALE, 1080 * SCALE);

// True if we've aligned a template and have data available
bool cloudAvailable;

// Used to transform from world space to bin space (or vice-versa)
tf::TransformListener* listener;

// Filter used for prefiltering the cloud (removes high luminosity)
pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;

// Strings for updating text and point clouds in the views and for naming windows
const std::string imageWindowName = "Kinect2Img";
const std::string linkText = "linkText";
const std::string pauseText = "pauseText";
const std::string linkCloudName = "linkCloud";
const std::string targetCloudName = "targetCloud";
const std::string templateCloudName = "templateCloud";
const std::string sceneCloudName = "sceneCloud";
const std::string sceneCloudMatchName = "sceneCloudMatch";
const std::string gripCloudName = "gripCloud";
const std::string fullLinkCloudName = "fullLinkCloud";

// Viewer parameters to ensure clouds are only added once
bool addOnce;
bool addSceneCloudOnce;

// Parameter server parameters
float x_range; // (Half the range)
float y_range; // (Half the range)
float z_range; // (Half the range)
int rMax;
int gMax;
int bMax;
int align_max_iterations;

// Keyboard checks (non-functional)
bool unpressedLeft;
bool unpressedRight;

// Current link displayed in the link viewer
int linkViewed;
const int linkX = 10;
const int linkY = 15;
const int linkFont = 15;

// Point resizing parameters for point clouds
const int pointSize = 3;
const int boldPointSize = 5;

// Whether to alignment process is paused or not
bool paused;

// Pause parameters and display settings
bool pauseOption; // If true, auto-pausing is enabled
int p; // Pause modulus (> 0)
int q; // Counter of alignments
const int pauseX = 5;
const int pauseY = 500;
const int pauseFont = 25;
bool publishMessageOnce;

// Set to true to exit
bool quit;

// Transform and index of best point to provide when the service is called and the time of the alignment
tf::StampedTransform transform;

// Put this at the top for quick reference and changes
// Call this first!
void SetUpParameters()
{
    //cloudSet = false;
    paused = false;
    cloudAvailable = false;
    linkViewed = 0;
    x_range = 0.3f; // (Half the range)
    y_range = 0.3f; // (Half the range)
    z_range = 1.75f; // (Half the range)
    rMax = 100;
    gMax = 100;
    bMax = 100;
    p = 1;
    q = 1;
    unpressedLeft = true;
    unpressedRight = true;
    addOnce = false;
    addSceneCloudOnce = false;
    publishMessageOnce = false;
}

//
// </Parameters>
// <ParamServer>
//

// Sets up the ROS param server with default values unless values are already written,
// In which case, it sets these values to those
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

// Checks for and updates parameters from the ROS param server
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

//
// </ParamServer>
// <Filtering>
//

// Removes points over a certain illuminacity and clips the search window
void FilterCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = cloud.makeShared();

    int s = cloud.size();
    printf("Initially filter cloud of size %d points... ", s);

    filter.setInputCloud(cloudPtr);
    filter.filter(cloud);

    printf("Removed %d yellow points.\n", (int)(s - cloud.size()));

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

//
// </Filtering>
// <Pausing>
//

// Pauses/unpauses the alignment procedure (allowing the views to be more easily used)
void Pause(bool pause)
{
    paused = pause;

    if (paused)
    {
        if (view) viewer->updateText("Paused", pauseX, pauseY, pauseFont, 1.0, 1.0, 1.0, pauseText);
        if (scene) sceneViewer->updateText("Paused", pauseX, pauseY, pauseFont, 1.0, 1.0, 1.0, pauseText);
        if (links) linkViewer->updateText("Paused", pauseX, pauseY, pauseFont, 1.0, 1.0, 1.0, pauseText);
        printf("Paused\n");
    }
    else
    {
        if (view) viewer->updateText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);
        if (scene) sceneViewer->updateText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);
        if (links) linkViewer->updateText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);
        printf("Unpaused\n");
    }
}

//
// </Pausing>
// <Keyboard Callbacks>
//

// Keyboard event callback for non-link views
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> eviewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "p")
  {
      Pause(!paused);
  }
  else if (event.getKeySym () == "q")
  {
      quit = true;
  }
  else { }
}

// Keyboard event callback for the link view
void linkKeyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* link_viewer_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> eviewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown())
  {
      if (event.getKeySym () == "p")
      {
          Pause(!paused);
      }
      else if (unpressedLeft && (event.getKeyCode() == 60 || event.getKeyCode() == 44)) // "<" || ","
      {
          linkViewed = (--linkViewed+aligner.GetNumTemplates())%aligner.GetNumTemplates();
          linkViewer->updateText("Link #" + boost::lexical_cast<std::string>(linkViewed), linkX, linkY, linkFont, 1.0, 1.0, 1.0, linkText);
          linkViewer->updatePointCloud(aligner.GetTemplate(linkViewed), linkCloudName);
          unpressedLeft = false;
      }
      else if (unpressedRight && (event.getKeyCode() == 62 || event.getKeyCode() == 46)) // ">" || "."
      {
          linkViewed = (++linkViewed+aligner.GetNumTemplates())%aligner.GetNumTemplates();
          linkViewer->updateText("Link #" + boost::lexical_cast<std::string>(linkViewed), 10, 10, linkText);
          linkViewer->updatePointCloud(aligner.GetTemplate(linkViewed), linkCloudName);
          unpressedRight = false;
      }
      else if (event.getKeySym () == "q")
      {
          ROS_INFO("Quitting");
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

//
// </Keyboard Callbacks>
// <ROS Callbacks>
//

// sensor_msgs/PointCloud2
void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
    if (!paused || !cloudAvailable)
    {
        pcl::fromROSMsg(*points, cloud);
        //pcl::fromROSMsg(*points, nextCloud);
        /*if (cloudSet)
        {
            cloud = previousCloud + nextCloud;
        }
        else
        {
            cloud = nextCloud;
            //cloudSet = true;
        }
        previousCloud = nextCloud;*/

        ROS_INFO("Cloud received");
        FilterCloud();
        aligner.SetTargetCloud(cloud.makeShared());

        aligner.Align();

        try
        {
            tf::StampedTransform world;

            std::string frameId;
            std::string parentId = "kinect2_rgb_optical_frame";
            if (!kinect)
            {
                frameId = "container";
            }
            else
            {
                frameId = parentId;
            }

            listener->lookupTransform(parentId, frameId,
                                   ros::Time(0), world);

            // Apply the inverse rotation from the header
            Eigen::Vector3f t;
            Eigen::Quaternionf r;

            //tf::vectorTFToEigen(world.getOrigin(), t);
            //tf::quaternionTFToEigen(world.getRotation(), r);

            float ww = world.getOrigin().w();
            t[0] = world.getOrigin().x() / ww;
            t[1] = world.getOrigin().y() / ww;
            t[2] = world.getOrigin().z() / ww;
            //t[3] = world.getOrigin().w();

            r.x() = world.getRotation().x();
            r.y() = world.getRotation().y();
            r.z() = world.getRotation().z();
            r.w() = world.getRotation().w();

            // Apply container transform to grip points
            pcl::PointCloud<pcl::PointXYZRGB> transformedGripCloud;
            pcl::transformPointCloud(gripCloud, transformedGripCloud, t, r);

            // Select one grip point (the one with the lowest z should always be the best)
            int index = 0;
            int minZ = transformedGripCloud.points[index].z;
            // Horribly ineffient way to check for valid rotations
            pcl::PointCloud<pcl::PointXYZ> zPoint;
            zPoint.width    = 1;
            zPoint.height   = 1;
            zPoint.is_dense = false;
            zPoint.points.resize (zPoint.width * zPoint.height);
            zPoint.points[0].x = 0.0f;
            zPoint.points[0].y = 0.0f;
            zPoint.points[0].z = 0.0f;
            // Best the best grasp point and rotation
            for (int i = 1; i < transformedGripCloud.points.size(); i++)
            {
                if (transformedGripCloud.points[i].z < minZ) // Check for the best position
                {
                    pcl::transformPointCloud(zPoint, zPoint, t, r);
                    if (zPoint[0].z > 0) // The grabber should point down
                    {
                        minZ = transformedGripCloud.points[i].z;
                        index = i;
                    }
                    else { }
                }
                else { }
            }

            // Link -> Container -> World
            Eigen::Matrix3f linkMat = GripRots[index];
            tf::StampedTransform link;

            float w = sqrt(1.0f + linkMat(0,0) + linkMat(1,1) + linkMat(2,2)) / 2.0f;
            float w4 = (4.0f * w);
            float x = (linkMat(2,1) - linkMat(1,2)) / w4;
            float y = (linkMat(0,2) - linkMat(2,0)) / w4;
            float z = (linkMat(1,0) - linkMat(0,1)) / w4;

            link.setOrigin(tf::Vector3(GripPoints[index][0], GripPoints[index][1], GripPoints[index][2]));
            link.setRotation(tf::Quaternion(x, y, z, w));

            ROS_INFO("Link:");
            ROS_INFO("Position x: %f, y: %f, z: %f",
                     link.getOrigin().getX(),
                     link.getOrigin().getY(),
                     link.getOrigin().getZ());
            ROS_INFO("Rotation x: %f, y: %f, z: %f, w: %f",
                     link.getRotation().getX(),
                     link.getRotation().getY(),
                     link.getRotation().getZ(),
                     link.getRotation().getW());

            // -> Container -> World
            Eigen::Matrix4f containerMat = aligner.GetBestTransform();
            tf::StampedTransform container;

            w = sqrt(1.0f + containerMat(0,0) + containerMat(1,1) + containerMat(2,2)) / 2.0f;
            w4 = (4.0f * w);
            x = (containerMat(2,1) - containerMat(1,2)) / w4;
            y = (containerMat(0,2) - containerMat(2,0)) / w4;
            z = (containerMat(1,0) - containerMat(0,1)) / w4;

            container.setOrigin(tf::Vector3(containerMat(0,3), containerMat(1,3), containerMat(2,3)));
            container.setRotation(tf::Quaternion(x, y, z, w));

            ROS_INFO("Container:");
            ROS_INFO("Position x: %f, y: %f, z: %f",
                     container.getOrigin().getX(),
                     container.getOrigin().getY(),
                     container.getOrigin().getZ());
            ROS_INFO("Rotation x: %f, y: %f, z: %f, w: %f",
                     container.getRotation().getX(),
                     container.getRotation().getY(),
                     container.getRotation().getZ(),
                     container.getRotation().getW());

            // -> World
            tf::Transform trans = container * link;
            //tf::Transform trans = link * container;
            transform.frame_id_ = frameId;
            transform.child_frame_id_ = chainLinkFrameId;
            transform.setOrigin(trans.getOrigin());
            transform.setRotation(trans.getRotation());

            cloudAvailable = true;
            publishMessageOnce = false;

            // Draw the grip axes in the picture
            pcl::PointCloud<pcl::PointXYZRGB> axes;
            axes.width    = 4;
            axes.height   = 1;
            axes.is_dense = false;
            axes.points.resize (axes.width * axes.height);

            axes.points[0].x = 0.0f;
            axes.points[0].y = 0.0f;
            axes.points[0].z = 0.0f;

            axes.points[1].x = 0.05f;
            axes.points[1].y = 0.0f;
            axes.points[1].z = 0.0f;

            axes.points[2].x = 0.0f;
            axes.points[2].y = 0.05f;
            axes.points[2].z = 0.0f;

            axes.points[3].x = 0.0f;
            axes.points[3].y = 0.0f;
            axes.points[3].z = 0.05f;

            pcl::transformPointCloud(axes, axes, t, r);

            pcl::PointCloud<pcl::PointXYZRGB> rotationCloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotationCloudShared = rotationCloud.makeShared();
            pcl::transformPointCloud(fullLinkCloud, rotationCloud, t, r);

            if (scene) // If the scene viewer is available
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fullLinkColor
                    (rotationCloudShared, 255, 255, 255);
                sceneViewer->updatePointCloud(rotationCloudShared, fullLinkColor, fullLinkCloudName);

                sceneViewer->removeShape("x");
                sceneViewer->removeShape("y");
                sceneViewer->removeShape("z");
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[1], 255, 255, 0, "x");
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[2], 0, 255, 255, "y");
                sceneViewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[3], 255, 0, 255, "z");

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgcShared = transformedGripCloud.makeShared();
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
                    (tgcShared, 255, 0, 0);
                sceneViewer->updatePointCloud(tgcShared, gripColor, gripCloudName);
            }
            else { }

            if (view)
            {
                /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> targetColor
                    (aligner.GetTargetCloud(), 0, 255, 255);*/
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> templateColor
                    (aligner.GetMostAlignedTemplate(), 0, 255, 0);
                if (!addOnce)
                {
                    viewer->addPointCloud(aligner.GetTargetCloud(), targetCloudName);
                    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, targetCloudName);
                    viewer->addPointCloud(aligner.GetMostAlignedTemplate(), templateColor, templateCloudName);
                    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, templateCloudName);
                    addOnce = true;
                }
                else
                {
                    viewer->updatePointCloud(aligner.GetTargetCloud(), targetCloudName);
                    viewer->updatePointCloud(aligner.GetMostAlignedTemplate(), templateColor, templateCloudName);
                }

                viewer->removeShape("x");
                viewer->removeShape("y");
                viewer->removeShape("z");
                viewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[1], 255, 255, 0, "x");
                viewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[2], 0, 255, 255, "y");
                viewer->addLine<pcl::PointXYZRGB> (axes.points[0], axes.points[3], 255, 0, 255, "z");
            }
            else { }

            q++;
            if (pauseOption && q%p == 0)
            {
                Pause(!paused);
            }
            else { }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }
    else { }
}

// sensor_msgs/PointCloud2
void sceneCallback(const sensor_msgs::PointCloud2ConstPtr& points)
{
    if (cloudAvailable && !paused)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> templateColor
            (aligner.GetMostAlignedTemplate(), 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fullLinkColor
            (fullLinkCloud.makeShared(), 255, 255, 255);

        ROS_INFO("Scene Received");

        try
        {
            if (!kinect)
            {
                sensor_msgs::PointCloud2 transformed;

                pcl_ros::transformPointCloud("/container", *points, transformed, *listener);

                ROS_INFO("Scene Transformed");

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
            sceneViewer->updatePointCloud(fullLinkCloud.makeShared(), fullLinkColor, fullLinkCloudName);
        }
    }
    else { }
}

// sensor_msgs/ImageConstPtr
void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::resize(cv_ptr->image, cv_ptr->image, windowSize);
      cv::imshow(imageWindowName, cv_ptr->image);
      cv::updateWindow(imageWindowName);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

// Publish the transform of the grip point
// geometry_msgs/PoseStamped
bool Publish(tf::TransformBroadcaster broadcaster)
{
    //printf("\n");
    //ROS_INFO("Publishing grip pose");

    if (!cloudAvailable)
    {
        //ROS_INFO("No grip ready yet");
        return -1;
    }
    else { }

    transform.stamp_ = ros::Time::now();

    broadcaster.sendTransform(transform);

    if (!publishMessageOnce)
    {
        printf("\n");
        ROS_INFO("Grip published:");
        ROS_INFO("Position x: %f, y: %f, z: %f",
                     transform.getOrigin().getX(),
                     transform.getOrigin().getY(),
                     transform.getOrigin().getZ());
            ROS_INFO("Rotation x: %f, y: %f, z: %f, w: %f",
                     transform.getRotation().x(),
                     transform.getRotation().y(),
                     transform.getRotation().z(),
                     transform.getRotation().w());
        printf("\n");
        publishMessageOnce = true;
    }
    else { }

    return true;
}

//
// </ROS Callbacks>
// <Main>
//

int main(int argc, char** argv)
{
    SetUpParameters();

    quit = false;
    kinect = false;
    view = false;
    scene = false;
    links = false;
    image = false;

    std::string dpi = defaultDPI;
    std::string topic = defaultTopic;

    char* templates;

    // Check if we have enough parameters
    if (argc < 2)
    {
        printf ("No templates file given!\n");
        printf ("Use ./chain_link_grip <templates.txt> [-a] [-k [<sd or qhd or hd>] or -t <topic>] [-l] [-s] [-i] [-p <interval>]\n");
        return (-1);
    }
    else { }

    // Print out help text if requested
    if (strcmp(argv[1], "-h") == 0)
    {
        printf(
               "%s\n%s\n\n%s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n%-3s %-13s %-20s\n\n",

               "chain_link_grip searches a bin containing chain links for good points and an angle for a robot to grip and extract a single chain link and returns that point and orientation.",
               "Parameters can be in any order.",
               "Parameters:",
               "", "-k [dpi]", "Use the Kinect data directly; dpi: hd, sd, or qhd (hd is default)",
               "", "-t <topic>", "Use a different point cloud Topic as the input specified by <topic>",
               "", "-a", "Show the Aligned point clouds used by the program to find an alignment",
               "", "-l", "Show the aligned chain Link template point clouds",
               "", "-s", "Show the match made in the full Scene",
               "", "-i", "Show the color scene as an Image",
               "", "-p <interval>", "Pauses the alignment procedure every <interval> matches (must be > 0)"
               );
        return 0;
    }
    else { }

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
        for (int i = 2; i < argc; i++)
        {
            /* Various input types */
            if (strcmp(argv[i], "-k") == 0)
            {
                kinect = true;
                if (argc > i+1 &&
                        (strcmp(argv[i], "hd") == 0 ||
                         strcmp(argv[i], "sd") == 0 ||
                         strcmp(argv[i], "qhd") == 0))
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
                links = true;
            }
            else if (strcmp(argv[i], "-s") == 0)
            {
                scene = true;
            }
            else if (strcmp(argv[i], "-i") == 0)
            {
                image = true;
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
                        if (p < 1) //if (p < 0)
                        {
                            //printf ("Inverval must be non-negative!\n");
                            printf ("Inverval must be > 0!\n");
                            return (-1);
                        }
                        /*else if (p == 0)
                        {
                            p = 1;
                            paused = true; // Need to set it up like this since we haven't made the views yet
                        }
                        else { }*/
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

    printf("Starting...\n");

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

    if (image)
    {
        subImg = n.subscribe<sensor_msgs::Image>("/kinect2/hd/image_color_rect", 100, imageCallback);
    }
    else { }

    if (scene)
    {
        subScene = n.subscribe<sensor_msgs::PointCloud2>("/kinect2/hd/points", 100, sceneCallback);
    }
    else { }

    // Subscribe to the point cloud for matching
    sub = n.subscribe<sensor_msgs::PointCloud2>(topic, 100, pointsCallback);

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

    // Realign all the template clouds correctly
    aligner.ApplyTransformations(Views);

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

    if (links)
    {
        linkViewer = new pcl::visualization::PCLVisualizer ("Link Clouds");
        linkViewer->initCameraParameters ();
        linkViewer->addCoordinateSystem (0.1);
        linkViewer->setBackgroundColor (0.25, 0.25, 0.25, 0);
        linkViewer->addText ("Link #" + boost::lexical_cast<std::string>(linkViewed), linkX, linkY, linkFont, 1.0, 1.0, 1.0, linkText);
        linkViewer->addPointCloud(aligner.GetTemplate(linkViewed), linkCloudName);
        linkViewer->registerKeyboardCallback (linkKeyboardEventOccurred, (void*)&linkViewer);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
            (gripCloud.makeShared(), 255, 0, 0);
        linkViewer->addPointCloud(gripCloud.makeShared(), gripColor, gripCloudName);
        linkViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, gripCloudName);

        // Draw all the grip rotations
        pcl::PointCloud<pcl::PointXYZRGB> axes;
        pcl::PointCloud<pcl::PointXYZRGB> axesRotated;
        axes.width    = 4;
        axes.height   = 1;
        axes.is_dense = false;
        axes.points.resize (axes.width * axes.height);

        axes.points[0].x = 0.0f;
        axes.points[0].y = 0.0f;
        axes.points[0].z = 0.0f;

        axes.points[1].x = 0.01f;
        axes.points[1].y = 0.0f;
        axes.points[1].z = 0.0f;

        axes.points[2].x = 0.0f;
        axes.points[2].y = 0.01f;
        axes.points[2].z = 0.0f;

        axes.points[3].x = 0.0f;
        axes.points[3].y = 0.0f;
        axes.points[3].z = 0.01f;

        for (int i = 0; i < grip_num; i++)
        {
            Eigen::Matrix4f trans;
            trans.block(0,0,3,3) << GripRots[i];
            trans.block(0,3,3,1) << GripPoints[i];
            trans.block(3,3,1,1) << 1;
            pcl::transformPointCloud(axes, axesRotated, trans);
            linkViewer->addLine<pcl::PointXYZRGB> (axesRotated.points[0], axesRotated.points[1], 255, 0, 0, "x" + boost::lexical_cast<std::string>(i));
            linkViewer->addLine<pcl::PointXYZRGB> (axesRotated.points[0], axesRotated.points[2], 0, 255, 0, "y" + boost::lexical_cast<std::string>(i));
            linkViewer->addLine<pcl::PointXYZRGB> (axesRotated.points[0], axesRotated.points[3], 0, 0, 255, "z" + boost::lexical_cast<std::string>(i));
        }
    }
    else { }

    if (scene)
    {
        sceneViewer = new pcl::visualization::PCLVisualizer ("Scene");
        sceneViewer->initCameraParameters ();
        sceneViewer->addCoordinateSystem (0.1);
        sceneViewer->setBackgroundColor (0.15, 0.15, 0.15, 0);
        sceneViewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&sceneViewer);

        const std::string &pcd_file = "./data/ChainLinkResizeCenter.pcd";
        if (pcl::io::loadPCDFile (pcd_file, fullLinkCloud) == -1)
        {
            printf ("Error loading file %s\n", pcd_file.c_str());
            return -1;
        }
        else { }

        // Add the matched template to the scene too
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> fullLinkColor
            (fullLinkCloud.makeShared(), 255, 255, 255);
        sceneViewer->addPointCloud(fullLinkCloud.makeShared(), fullLinkColor, fullLinkCloudName);
        sceneViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, fullLinkCloudName);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> gripColor
            (gripCloud.makeShared(), 255, 0, 0);
        sceneViewer->addPointCloud(gripCloud.makeShared(), gripColor, gripCloudName);
        sceneViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, boldPointSize, gripCloudName);
    }
    else { }

    if (view) viewer->addText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);
    if (scene) sceneViewer->addText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);
    if (links) linkViewer->addText(".", pauseX, pauseY, 0, 1.0, 1.0, 1.0, pauseText);

    printf("Starting topic %s...\n", topicName.c_str());

    // Set up the service to provide grip points
    /*ros::Publisher publisher;
    publisher = n.advertise<geometry_msgs::PoseStamped>(topicName, 10);
    if (!publisher)
    {
        ROS_ERROR("Unable to start publisher");
        return -1;
    }
    else { }*/
    // Pose publisher
    tf::TransformBroadcaster broadcaster;

    // Now that the views are set up, we can properly pause if we need to
    /*if (paused) // Broken
    {
        Pause(paused);
    }
    else { }*/

    printf("Running...\n");

    // Main loop
    do
    {
        // Check for changed parameters
        HandleParams(n);

        // Update all the ROS callbacks
        ros::spinOnce();

        // Publish the grip pose
        //Publish(publisher);
        Publish(broadcaster);

        // Update the views
        if (view) viewer->spinOnce();
        if (links) linkViewer->spinOnce();
        if (scene) sceneViewer->spinOnce();
    }
    while (ros::ok() && !quit);

    // Begin shutdown
    if (view) delete viewer;
    if (links) delete linkViewer;
    if (scene) delete sceneViewer;

    delete listener;

    printf("Stopped\n");

    return 0;
}

//
// </Main>
//

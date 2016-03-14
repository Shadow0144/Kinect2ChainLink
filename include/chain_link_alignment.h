#ifndef CHAIN_LINK_ALIGNMENT_H_
#define CHAIN_LINK_ALIGNMENT_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchMethod;

class FeatureCloud
{
  public:
    FeatureCloud ();
    ~FeatureCloud ();
    // Process the given cloud
    void setInputCloud (PointCloud::Ptr xyz);
    // Load and process the cloud in the given PCD file
    void loadInputCloud (const std::string &pcd_file);
    // Get a pointer to the cloud 3D points
    PointCloud::Ptr getPointCloud () const;
    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr getSurfaceNormals () const;
    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr getLocalFeatures () const;
    // Downsamples the point cloud
    void DownSample(float voxel_grid_size);
    // Clip the point cloud in x, y, and z
    void Clip(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    // Clip the point cloud in z
    void Clip(float zMin, float zMax);
    // Translate the cloud
    void Translate(float x, float y, float z);
    // Rotate the cloud around the x axis
    void RotateX(float theta);
    // Rotate the cloud around the y axis
    void RotateY(float theta);
    // Rotate the cloud around the z axis
    void RotateZ(float theta);

    // Compute the surface normals and local features
    void processInput ();

  protected:
    // Compute the surface normals
    void computeSurfaceNormals ();
    // Compute the local feature descriptors
    void computeLocalFeatures ();

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment ();
    ~TemplateAlignment ();
    // Set the given cloud as the target to which the templates will be aligned
    void setTargetCloud (FeatureCloud &target_cloud);
    // Get the target cloud
    FeatureCloud getTargetCloud();
    // Add the given cloud to the list of template clouds
    void addTemplateCloud (FeatureCloud &template_cloud);
    // Adjust the number of max iterations
    void SetMaxIterations(int nr_iterations);
    // Get the number of max iterations
    int GetMaxIterations();
    // Align the given template cloud to the target specified by setTargetCloud ()
    void align (FeatureCloud &template_cloud, TemplateAlignment::Result &result);
    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results);
    // Align all of template clouds to the target cloud to find the one with best alignment score
    int findBestAlignment (TemplateAlignment::Result &result);

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

class TemplateAligner
{
public:
    TemplateAligner();
    TemplateAligner(char* objectTemplatesFile, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    TemplateAligner(char* objectTemplatesFile, char* targetCloudFile, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    TemplateAligner(char* objectTemplatesFile);
    TemplateAligner(char* objectTemplatesFile, char* targetCloudFile);
    ~TemplateAligner();
    void SetTargetCloud(PointCloud::Ptr xyz);
    PointCloud::Ptr GetTargetCloud();
    int GetNumTemplates();
    PointCloud::Ptr GetTemplate(int n);
    std::vector<PointCloud::Ptr> GetAlignedTemplates();
    PointCloud::Ptr GetMostAlignedTemplate();
    float GetTargetVoxelGridSize();
    void SetTargetVoxelGridSize(float size);
    float GetTemplateVoxelGridSize();
    void SetTemplateVoxelGridSize(float size);
    void SetTargetWindow(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    // Adjust the number of max iterations
    void SetMaxIterations(int nr_iterations);
    // Get the number of max iterations
    int GetMaxIterations();
    //void SetDisplayEnabled(bool enabled);
    //bool GetDisplayEnabled();
    void SetSaveEnabled(bool enabled);
    bool GetSaveEnabled();
    void SetOutputEnabled(bool enabled);
    bool GetOutputEnabled();
    void Align(); // Find best object template aligned to the target scene
    void AlignAll(); // Make all object templates aligned to the target scene

    // Getters for the window
    float GetxMinTarget() { return xMinTarget; }
    float GetxMaxTarget() { return xMaxTarget; }
    float GetyMinTarget() { return yMinTarget; }
    float GetyMaxTarget() { return yMaxTarget; }
    float GetzMinTarget() { return zMinTarget; }
    float GetzMaxTarget() { return zMaxTarget; }

private:
    TemplateAlignment template_align;
    std::vector<FeatureCloud> object_templates; // Templates to search for
    FeatureCloud target_cloud; // Align templates to this (Redundant with template_align?)
    std::vector<PointCloud::Ptr> aligned_templates;
    PointCloud::Ptr best_cloud; // Most aligned template
    void PreprocessTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    //bool displayEnabled = false;
    bool saveEnabled = false;
    bool outputEnabled = true;
    int numTemplates;
    bool targetLoaded = false;
    pcl::PCDWriter writer;
    // Target
    float voxel_grid_sizeTarget = 0.0005f; // 0.005f
    float xMinTarget = -10.5f;
    float xMaxTarget = +10.5f;
    float yMinTarget = -10.5f;
    float yMaxTarget = +10.5f;
    float zMinTarget = -10.0f;//+0.01f;
    float zMaxTarget = 10.0f;
    // Templates
    float voxel_grid_sizeTemplate = 0.0005f; // 0.005f
    //static xMinTemplate = -0.25f;
    //static xMaxTemplate = +0.25f;
    //static yMinTemplate = -0.25f;
    //static yMaxTemplate = +0.25f;
    float zMinTemplate = -5.0f;
    float zMaxTemplate = +0.0f;
    float xOffset = -0.765f;
    float yOffset = 0.0f;
    float zOffset = 0.0f;
    float xRot = 0.0f;
    float yRot = -0.5f * M_PI;
    float zRot = 0.0f;
};

#endif /* CHAIN_LINK_ALIGNMENT_H_ */

#include <chain_link_alignment.h>
#include "std_msgs/String.h"
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

//
//
// # FeatureCloud
//
//

// A bit of shorthand
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZRGB> SearchMethod;

FeatureCloud::FeatureCloud () :
  search_method_xyz_ (new SearchMethod),
  normal_radius_ (0.02f),
  feature_radius_ (0.02f)
{ }

FeatureCloud::~FeatureCloud () { }

// Process the given cloud
void
FeatureCloud::setInputCloud (PointCloud::Ptr xyz)
{
  xyz_ = xyz;
}

// Load and process the cloud in the given PCD file
void
FeatureCloud::loadInputCloud (const std::string &pcd_file)
{
  xyz_ = PointCloud::Ptr (new PointCloud);
  if (pcl::io::loadPCDFile (pcd_file, *xyz_) == -1)
  {
      printf ("Error loading file %s\n", pcd_file.c_str());
      return;
  }
  else { }
}

// Downsamples the point cloud
void
FeatureCloud::DownSample(float voxel_grid_size)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setInputCloud (xyz_);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    vox_grid.filter (*tempCloud);
    xyz_ = tempCloud;
}

// Clip the point cloud in x, y, and z
void
FeatureCloud::Clip(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (xyz_);

    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xMin, xMax);
    pass.filter (*xyz_);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (yMin, yMax);
    pass.filter (*xyz_);

    pass.setFilterFieldName ("z");
    pass.setFilterLimits (zMin, zMax);
    pass.filter (*xyz_);
}

// Clip the point cloud in z
void
FeatureCloud::Clip(float zMin, float zMax)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (xyz_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (zMin, zMax);
    pass.filter (*xyz_);
}

// Translate the cloud
void
FeatureCloud::Translate(float x, float y, float z)
{
    Eigen::Matrix4f transform;
    transform <<
            1.0f, 0.0f, 0.0f, x,
            0.0f, 1.0f, 0.0f, y,
            0.0f, 0.0f, 1.0f, z,
            0.0f, 0.0f, 0.0f, 1.0f;
    pcl::transformPointCloud(*xyz_, *xyz_, transform);
}

// Rotate the cloud around the x axis
void
FeatureCloud::RotateX(float theta)
{
    float cosT = cosf(theta);
    float sinT = sinf(theta);
    Eigen::Matrix4f transform;
    transform <<
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, cosT, -sinT, 0.0f,
            0.0f, sinT, cosT, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
    pcl::transformPointCloud(*xyz_, *xyz_, transform);
}

// Rotate the cloud around the y axis
void
FeatureCloud::RotateY(float theta)
{
    float cosT = cosf(theta);
    float sinT = sinf(theta);
    Eigen::Matrix4f transform;
    transform <<
            cosT, 0.0f, sinT, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            -sinT, 0.0f, cosT, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
    pcl::transformPointCloud(*xyz_, *xyz_, transform);
}

// Rotate the cloud around the z axis
void
FeatureCloud::RotateZ(float theta)
{
    float cosT = cosf(theta);
    float sinT = sinf(theta);
    Eigen::Matrix4f transform;
    transform <<
            cosT, -sinT, 0.0f, 0.0f,
            sinT, cosT, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;
    pcl::transformPointCloud(*xyz_, *xyz_, transform);
}

// Applies the transform
void
FeatureCloud::Transform(Eigen::Matrix4f transform)
{
    pcl::transformPointCloud(*xyz_, *xyz_, transform);
}

// Get a pointer to the cloud 3D points
PointCloud::Ptr
FeatureCloud::getPointCloud () const
{
  return (xyz_);
}

// Get a pointer to the cloud of 3D surface normals
SurfaceNormals::Ptr
FeatureCloud::getSurfaceNormals () const
{
  return (normals_);
}

// Get a pointer to the cloud of feature descriptors
LocalFeatures::Ptr
FeatureCloud::getLocalFeatures () const
{
  return (features_);
}

// Compute the surface normals and local features
void
FeatureCloud::processInput ()
{
  std::vector<int> indices;
  // Remove any invalid points
  pcl::removeNaNFromPointCloud(*xyz_,*xyz_,indices);
  computeSurfaceNormals ();
  computeLocalFeatures ();
}

// Compute the surface normals
void
FeatureCloud::computeSurfaceNormals ()
{
  normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  norm_est.setInputCloud (xyz_);
  norm_est.setSearchMethod (search_method_xyz_);
  norm_est.setRadiusSearch (normal_radius_);
  norm_est.compute (*normals_);
}

// Compute the local feature descriptors
void
FeatureCloud::computeLocalFeatures ()
{
  features_ = LocalFeatures::Ptr (new LocalFeatures);

  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (xyz_);
  fpfh_est.setInputNormals (normals_);
  fpfh_est.setSearchMethod (search_method_xyz_);
  fpfh_est.setRadiusSearch (feature_radius_);
  fpfh_est.compute (*features_);
}

//
//
// # TemplateAlignment
//
//

TemplateAlignment::TemplateAlignment () :
  min_sample_distance_ (minSampleDistance),
  max_correspondence_distance_ (maxCorrespondenceDistance),
  nr_iterations_ (numIterations) // 500
{
  // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
  sac_ia_.setMinSampleDistance (min_sample_distance_);
  sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
  sac_ia_.setMaximumIterations (nr_iterations_);
}

TemplateAlignment::~TemplateAlignment () { }

// Set the given cloud as the target to which the templates will be aligned
void
TemplateAlignment::setTargetCloud (FeatureCloud &target_cloud)
{
  target_ = target_cloud;
  sac_ia_.setInputTarget (target_cloud.getPointCloud ());
  sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
}

// Get the target cloud
FeatureCloud
TemplateAlignment::getTargetCloud()
{
    return target_;
}

// Add the given cloud to the list of template clouds
void
TemplateAlignment::addTemplateCloud (FeatureCloud &template_cloud)
{
  templates_.push_back (template_cloud);
}

// Adjust the number of max iterations
void
TemplateAlignment::SetMaxIterations(int nr_iterations)
{
    nr_iterations_ = nr_iterations;
    sac_ia_.setMaximumIterations (nr_iterations_);
}

// Get the number of max iterations
int
TemplateAlignment::GetMaxIterations()
{
    return nr_iterations_;
}

// Align the given template cloud to the target specified by setTargetCloud ()
void
TemplateAlignment::align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
{
  sac_ia_.setInputSource (template_cloud.getPointCloud ());
  sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

  pcl::PointCloud<pcl::PointXYZRGB> registration_output;
  sac_ia_.align (registration_output);

  result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
  result.final_transformation = sac_ia_.getFinalTransformation ();
}

// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
void
TemplateAlignment::alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
{
  results.resize (templates_.size ());
  for (size_t i = 0; i < templates_.size (); ++i)
  {
    align (templates_[i], results[i]);
  }
}

// Align all of template clouds to the target cloud to find the one with best alignment score
int
TemplateAlignment::findBestAlignment (TemplateAlignment::Result &result)
{
  // Align all of the templates to the target cloud
  std::vector<Result, Eigen::aligned_allocator<Result> > results;
  alignAll (results);

  // Find the template with the best (lowest) fitness score
  float lowest_score = std::numeric_limits<float>::infinity ();
  int best_template = 0;
  for (size_t i = 0; i < results.size (); ++i)
  {
    const Result &r = results[i];
    if (r.fitness_score < lowest_score)
    {
      lowest_score = r.fitness_score;
      best_template = (int) i;
    }
  }

  // Output the best alignment
  result = results[best_template];
  return (best_template);
}

// Apply a series of transformations to each of the FeatureClouds
void
TemplateAlignment::ApplyTransformations(Eigen::Matrix4f transforms[])
{
    int i = 0;
    for (std::vector<FeatureCloud>::iterator it = templates_.begin();
         it != templates_.end();
         ++it)
    {
        ((FeatureCloud)*it).Transform(transforms[i++]);
    }
}

//
//
// # TemplateAligner
//
//

// Base constructor
TemplateAligner::TemplateAligner(char* objectTemplatesFile,
                                 float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    Construct(objectTemplatesFile, xMin, xMax, yMin, yMax, zMin, zMax);
}

TemplateAligner::TemplateAligner(char* objectTemplatesFile, char* targetCloudFile,
                                 float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    Construct(objectTemplatesFile, xMin, xMax, yMin, yMax, zMin, zMax);

    // Load the target cloud PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile (targetCloudFile, *cloud) == -1)
    {
        printf ("Error loading file %s\n", targetCloudFile);
        return;
    }
    else { }

    PreprocessTargetCloud(cloud);
}

TemplateAligner::TemplateAligner(char* objectTemplatesFile, char* targetCloudFile)
{
    Construct(objectTemplatesFile, xMinTarget, xMaxTarget, yMinTarget, yMinTarget, zMinTarget, zMaxTarget);

    // Load the target cloud PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile (targetCloudFile, *cloud) == -1)
    {
        printf ("Error loading file %s\n", targetCloudFile);
        return;
    }
    else { }

    PreprocessTargetCloud(cloud);
}

TemplateAligner::TemplateAligner(char* objectTemplatesFile)
{
    Construct(objectTemplatesFile, xMinTarget, xMaxTarget, yMinTarget, yMinTarget, zMinTarget, zMaxTarget);
}

TemplateAligner::TemplateAligner() { }

void
TemplateAligner::Construct(char* objectTemplatesFile, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    SetParams();
    setbuf(stdout, NULL);

    // Load the object templates specified in the object_templates.txt file
    std::ifstream input_stream (objectTemplatesFile); //(argv[1]);
    object_templates.resize (0);
    std::string pcd_filename;
    int i = 1;
    while (input_stream.good ())
    {
      std::getline (input_stream, pcd_filename);
      if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
        continue;

      if (outputEnabled)
      {
          printf("Loading template %d... ", i++);
      }
      else { }
      FeatureCloud template_cloud;
      template_cloud.loadInputCloud (pcd_filename);
      template_cloud.DownSample(voxel_grid_sizeTemplate);
      //template_cloud.Translate(xOffset, yOffset, zOffset); // Bring back to center
      //template_cloud.RotateX(xRot); // Rotate to correct angle
      //template_cloud.RotateY(yRot); // Rotate to correct angle
      //template_cloud.RotateZ(zRot); // Rotate to correct angle
      //template_cloud.Clip(zMinTemplate, zMaxTemplate); // Clip templates along z!
      template_cloud.processInput();

      object_templates.push_back (template_cloud);
      if (outputEnabled)
      {
          printf("Template loaded\n");
      }
      else { }
    }
    input_stream.close ();

    numTemplates = --i;

    // Set the TemplateAlignment inputs
    for (size_t i = 0; i < object_templates.size (); ++i)
    {
        template_align.addTemplateCloud (object_templates[i]);
    }

    // Target culling window (slightly redundant for some constructors TODO)
    xMinTarget = xMin; xMaxTarget = xMax;
    yMinTarget = yMin; yMaxTarget = yMax;
    zMinTarget = zMin; zMaxTarget = zMax;
}

TemplateAligner::~TemplateAligner() { }

void
TemplateAligner::PreprocessTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Assign to the target FeatureCloud
    if (outputEnabled)
    {
        printf("Loading cloud of size %d points... ", ((int)cloud->size()));
    }
    else { }
    target_cloud.setInputCloud (cloud);
    //target_cloud.DownSample(voxel_grid_sizeTarget);
    target_cloud.Clip(xMinTarget, xMaxTarget, yMinTarget, yMaxTarget, zMinTarget, zMaxTarget);
    target_cloud.processInput();

    template_align.setTargetCloud (target_cloud);
    if (outputEnabled)
    {
        printf("Loaded cloud of size %d points.\n", ((int)target_cloud.getPointCloud()->size()));
    }
    else { }

    targetLoaded = true;
}

float
TemplateAligner::GetTargetVoxelGridSize()
{
    return voxel_grid_sizeTarget;
}

void
TemplateAligner::SetTargetVoxelGridSize(float size)
{
    voxel_grid_sizeTarget = size;
    target_cloud.DownSample(voxel_grid_sizeTarget);
}

float
TemplateAligner::GetTemplateVoxelGridSize()
{
    return voxel_grid_sizeTemplate;
}

void
TemplateAligner::SetTemplateVoxelGridSize(float size)
{
    voxel_grid_sizeTemplate = size;
    target_cloud.DownSample(voxel_grid_sizeTemplate);
}

void
TemplateAligner::SetTargetWindow(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    xMinTarget = xMin; xMaxTarget = xMax;
    yMinTarget = yMin; yMaxTarget = yMax;
    zMinTarget = zMin; zMaxTarget = zMax;
}

// Apply a series of transformations to each of the FeatureClouds
void
TemplateAligner::ApplyTransformations(Eigen::Matrix4f transforms[])
{
    template_align.ApplyTransformations(transforms);
}

// Adjust the number of max iterations
void
TemplateAligner::SetMaxIterations(int nr_iterations)
{
    template_align.SetMaxIterations (nr_iterations);
}

// Get the number of max iterations
int
TemplateAligner::GetMaxIterations()
{
    return template_align.GetMaxIterations();
}

void
TemplateAligner::SetTargetCloud(PointCloud::Ptr xyz)
{
    PreprocessTargetCloud(xyz);

    if (saveEnabled)
    {
        writer.writeBinary("filter.pcd", *xyz);
    }
    else { }
}

PointCloud::Ptr
TemplateAligner::GetTargetCloud()
{
    return target_cloud.getPointCloud();
}

int
TemplateAligner::GetNumTemplates()
{
    return numTemplates;
}

PointCloud::Ptr
TemplateAligner::GetTemplate(int n)
{
    return object_templates[n].getPointCloud();
}

std::vector<PointCloud::Ptr>
TemplateAligner::GetAlignedTemplates()
{
    return aligned_templates;
}

PointCloud::Ptr
TemplateAligner::GetMostAlignedTemplate()
{
    return best_cloud;
}

Eigen::Matrix4f
TemplateAligner::GetBestTransform()
{
    return best_transform;
}

int
TemplateAligner::GetMostAlignedTemplateIndex()
{
    return best_index;
}

bool
TemplateAligner::GetSaveEnabled()
{
    return saveEnabled;
}

void
TemplateAligner::SetSaveEnabled(bool enabled)
{
    saveEnabled = enabled;
}

/*bool
TemplateAligner::GetDisplayEnabled()
{
    return displayEnabled;
}

void
TemplateAligner::SetDisplayEnabled(bool enabled)
{
    displayEnabled = enabled;
}*/

bool
TemplateAligner::GetOutputEnabled()
{
    return outputEnabled;
}

void
TemplateAligner::SetOutputEnabled(bool enabled)
{
    outputEnabled = enabled;
}

// Find best object template aligned to the target scene
void
TemplateAligner::Align()
{
    if (outputEnabled)
    {
        printf("\n");
        printf("Aligning...\n");
        printf("\n");
    }
    else { }

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    best_index = template_align.findBestAlignment (best_alignment);
    const FeatureCloud &best_template = object_templates[best_index];

    if (outputEnabled)
    {
        // Print the alignment fitness score (values less than 0.00002 are good)
        printf ("Best fitness score: %f\n", best_alignment.fitness_score);
    }
    else { }

    // Print the rotation matrix and translation vector
    best_transform = best_alignment.final_transformation;
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

    if (outputEnabled)
    {
        printf ("\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("t = < %0.3f, %0.3f, %0.3f >\n\n", translation (0), translation (1), translation (2));
    }
    else { }

    // Save the aligned template for visualization
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
    best_cloud = transformed_cloud.makeShared();

    if (saveEnabled)
    {
        pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);
    }
    else { }
}

// Make all object templates aligned to the target scene
void
TemplateAligner::AlignAll()
{
    if (outputEnabled)
    {
        printf("\n");
        printf("Aligning...\n");
        printf("\n");
    }
    else { }

    // Find the best template alignment
    std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<TemplateAlignment::Result> > results =
            std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<TemplateAlignment::Result> >();
    template_align.alignAll(results);
    aligned_templates.clear();

    for (int i = 0; i < results.size(); i++)
    {
        //int index = //
        const FeatureCloud &itemplate = object_templates[i];
        TemplateAlignment::Result alignment = results[i];

        if (outputEnabled)
        {
            // Print the alignment fitness score (values less than 0.00002 are good)
            printf ("\nFitness score %d: %f\n", i, alignment.fitness_score);
        }
        else { }

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = alignment.final_transformation.block<3,3>(0, 0);
        Eigen::Vector3f translation = alignment.final_transformation.block<3,1>(0, 3);

        if (outputEnabled)
        {
            printf ("\n");
            printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
            printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
            printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
            printf ("\n");
            printf ("t = < %0.3f, %0.3f, %0.3f >\n\n", translation (0), translation (1), translation (2));
        }
        else { }

        // Save the aligned template for visualization
        pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
        pcl::transformPointCloud (*itemplate.getPointCloud (), transformed_cloud, alignment.final_transformation);
        aligned_templates.push_back(transformed_cloud.makeShared());

        if (saveEnabled)
        {
            pcl::io::savePCDFileBinary ("output" + boost::lexical_cast<std::string>(i) + ".pcd", transformed_cloud);
        }
        else { }
    }
}


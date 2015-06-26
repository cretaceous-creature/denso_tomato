#ifndef TOMATOSEGMENTATION_H
#define TOMATOSEGMENTATION_H


#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <sstream>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
//sphere fitting
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>

using namespace std;
using namespace pcl;

union ParamXYZ
{
    float data[3];
    struct {
        float x;
        float y;
        float z;
    };
};

union ParamDist
{
    float data[6];
    struct {
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        float min_z;
        float max_z;
    };
};
/*
union ParamSeg
{
    std::string *Segparam;
    struct{
        float Radius_min;
        float Radius_max;
        float Dist_weight;
        float Dist_threshold;
    };
};
*/

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointT1;
typedef pcl::PointXYZRGB Pointhandeye;

struct SegModels
{
    pcl::ModelCoefficients ModelCoeff;
    pcl::PointCloud<PointT>::Ptr Set_cloud;
};


//process

class TOMATOSEGMENTATION
{
public:
    explicit TOMATOSEGMENTATION();
    ~TOMATOSEGMENTATION();

public:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     std::string Param, std::string Parambox);

      pcl::PointCloud<Pointhandeye>::Ptr
      HandCloudFilter(pcl::PointCloud<Pointhandeye>::Ptr cloud);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                     pcl::PointCloud<Pointhandeye>::Ptr cloudframe);
        //fiting the cloud use polynomial
      pcl::PointCloud<PointT1>::Ptr
      Polyfit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        //segmentation
      int Sphere_seg(pcl::PointCloud<PointT1>::Ptr cloud);
      vector<SegModels> Tomato_model;
      Eigen::Matrix4d transformation_matrix;
private:
      ParamXYZ XYZcenter;
      ParamDist Distfilter;
      std::string SegmentParam;
      //RGB and Distance, return the number of points.
      unsigned int Distancefilter();
      void Transformtozero();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout;

};


#endif

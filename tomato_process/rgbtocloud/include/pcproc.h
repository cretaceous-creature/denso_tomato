#ifndef PCPROC_H
#define PCPROC_H


#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <sstream>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace pcl;
#define track_weight 0.1

union ParamRGB
{
    int data[3];
    struct {
        int r;
        int g;
        int b;
    };
};

union ParamDist
{
    float data[2];
    struct {
        float min;
        float max;
    };
};

union ParamNoise
{
   double data[2];
   struct {
        int meank;
        double Thresh;
   };

};

union HSI_SPACE
{
    int data[3];
    struct{
        int H;
        int S;
        int I;
    };
};


//process
class TOMATODETECT
{
public:
    explicit TOMATODETECT();
    ~TOMATODETECT();

public:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      HandCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
      int GetCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
      float Removeoutliner(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           Eigen::Vector4f *centroid);
      std::vector<Eigen::Vector4f> Center_vector;
      std::vector<pcl::ModelCoefficients> Boundingbox;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     std::string Param);
private:
      ParamRGB RGBfilterH;
      ParamRGB RGBfilterL;
      ParamDist Distfilter;
      ParamNoise Noisefilter;
      HSI_SPACE Assign_HSI(pcl::PointXYZRGB p);
      //RGB and Distance, return the number of points.
      unsigned int RGBDfilter();
      unsigned int NoiseFilter();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin;
      unsigned int PCsize;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout;
      Eigen::Vector4f LastCenter;
      int Tracking_flag;
};


#endif

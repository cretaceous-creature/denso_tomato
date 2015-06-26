#ifndef PCPROC_H
#define PCPROC_H


#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <sstream>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

using namespace std;
using namespace pcl;

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


//process
class BOUNDARYPROC
{
public:
    explicit BOUNDARYPROC();
    ~BOUNDARYPROC();

public:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
       CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     std::string Param);

private:
      ParamRGB RGBfilterH;
      ParamRGB RGBfilterL;
      ParamDist Distfilter;
      //RGB and Distance, return the number of points.
      unsigned int RGBDfilter();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin;
      unsigned int PCsize;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout;
};


#endif

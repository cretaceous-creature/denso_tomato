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
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>


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


//process
class RECONPROC
{
public:
    explicit RECONPROC();
    ~RECONPROC();

public:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     std::string Param, std::string Parambox);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudframe);
private:
      ParamXYZ XYZcenter;

      ParamDist Distfilter;
      //RGB and Distance, return the number of points.
      unsigned int Distancefilter();
      void Transformtozero();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout;

};


#endif

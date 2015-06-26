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

#include <ros/ros.h>

using namespace std;
using namespace pcl;

union ParamXYZ
{
    float data[4];
    struct {
        float x;
        float y;
        float z;
        float radius;
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

typedef struct bd{
    ParamXYZ vector;
    float weight;
}BOARDER;


typedef struct ModelByTrack{
    pcl::ModelCoefficients Modelcoeff;
    int Appeartimes;
    vector<ParamXYZ> forcevector;
    vector<BOARDER> BV;
    BOARDER Pedicel_e;
    float orderdist;  //dist compare to ref point,
    //the less, the higher order
}MBT;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointT1;

#define loge  2.718282
#define PI  3.1415927
//process

class MODELING
{
public:
    explicit MODELING();
    ~MODELING();

public:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudframe);
      vector<ModelByTrack> Tomato_model;
      ParamXYZ G_vector;
      void Vector_Normalize(ParamXYZ *vector,float length = 1);
private:
      ParamXYZ XYZcenter;
      Eigen::Vector4f centroid;
      ParamDist Distfilter;
      //RGB and Distance, return the number of points.
      unsigned int Distancefilter();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout;
      bool Return_up(ParamXYZ vector);  
      void Assignforce();
      void Assignboarder();
      void Assignpedicel_e();
      float ReturnGauss(float length);
      float u,sig;

};


#endif

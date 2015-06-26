//jsk............chen

#include <icp_registration.h>

RECONPROC::RECONPROC()
{
}
RECONPROC::~RECONPROC()
{
}

//CloudIn
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
RECONPROC::CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string Param, std::string Parambox)
{
    //decode the param
    cloudin = cloud;
    std::vector<int> indices;
    cloudout = cloud->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    if(cloudout->points.size()<8)
        return cloudin->makeShared();
    istringstream in(Param);
    in>>XYZcenter.x>>XYZcenter.y>>XYZcenter.z;
    float bx,by,bz;
    istringstream in1(Parambox);
    in1>>bx>>by>>bz;

    Distfilter.min_x = XYZcenter.x - bx;
    Distfilter.max_x = XYZcenter.x + bx;
    Distfilter.min_y = XYZcenter.y - by;
    Distfilter.max_y = XYZcenter.y + by;
    Distfilter.min_z = XYZcenter.z - bz;
    Distfilter.max_z = XYZcenter.z + bz;

    int size = this->Distancefilter();
    this->Transformtozero();
    return cloudout->makeShared();
}

unsigned int RECONPROC::Distancefilter()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dist (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::PassThrough<pcl::PointXYZRGB> pass_through;
       // set input cloud
       pass_through.setInputCloud (cloudin);
       std::vector<int> indices;
       cloudout = cloudin->makeShared();
       pcl::removeNaNFromPointCloud(*cloudin,*cloudout, indices);
       if(cloudout->points.size()<8)
           return 0;
       // set fieldname we want to filter over
       pass_through.setFilterFieldName ("x");
       // set range for selected field to ~~~~~~ meters
       pass_through.setFilterLimits (Distfilter.min_x,Distfilter.max_x);
       // do filtering
       pass_through.filter (*cloud_dist);
       //y
       pass_through.setInputCloud (cloud_dist);
       pass_through.setFilterFieldName ("y");
       pass_through.setFilterLimits (Distfilter.min_y,Distfilter.max_y);
       pass_through.filter (*cloud_dist);
       //x
       pass_through.setInputCloud (cloud_dist);
       pass_through.setFilterFieldName ("z");
       pass_through.setFilterLimits (Distfilter.min_z,Distfilter.max_z);
       pass_through.filter (*cloud_dist);

       cloudout = cloud_dist->makeShared();
       return cloudout->points.size();
}

void RECONPROC::Transformtozero()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = 0; // The angle of rotation in radians
    transform (0,0) = cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = cos (theta);
    transform (0,3) = -XYZcenter.x;
    transform (1,3) = -XYZcenter.y;
    transform (2,3) = -XYZcenter.z;

    transform (3,3) = 1;
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZRGB>);
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloudout, *cloud_transform, transform);
    cloudout = cloud_transform->makeShared();
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
RECONPROC::ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudframe)
{
    //if first time
    if(cloudall->points.size()==0)
        return cloudframe->makeShared();
    //ICP
    if(cloudframe->points.size()==0)
        return cloudall->makeShared();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputCloud(cloudframe);
    icp.setInputTarget(cloudall);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    if(icp.hasConverged())
        return Final.makeShared();
    else
        return cloudall->makeShared();
}





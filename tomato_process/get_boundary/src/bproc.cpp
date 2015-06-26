//jsk............chen

#include <bproc.h>

BOUNDARYPROC::BOUNDARYPROC()
{
}
BOUNDARYPROC::~BOUNDARYPROC()
{
}
//CloudIn
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
BOUNDARYPROC::CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string Param)
{
    //decode the param
    cloudin = cloud;
    std::vector<int> indices;
    cloudout = cloud->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    if(cloudout->points.size()<8)
        return cloudin->makeShared();
    PCsize = cloudin->points.size();
    istringstream in(Param);
    in>>RGBfilterL.r>>RGBfilterH.r
            >>RGBfilterL.g>>RGBfilterH.g
            >>RGBfilterL.b>>RGBfilterH.b
            >>Distfilter.min_x>>Distfilter.max_x
            >>Distfilter.min_y>>Distfilter.max_y
            >>Distfilter.min_z>>Distfilter.max_z;
    //pcl::removeNaNFromPointCloud(*cloudin,*cloudin, indices);
    int size = this->RGBDfilter();
    return cloudout->makeShared();
}

unsigned int BOUNDARYPROC::RGBDfilter()
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
//then color filtering.,..........................................................................
       int rMax = RGBfilterH.r;
       int rMin = RGBfilterL.r;
       int gMax = RGBfilterH.g;
       int gMin = RGBfilterL.g;
       int bMax = RGBfilterH.b;
       int bMin = RGBfilterL.b;
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dist_color (new pcl::PointCloud<pcl::PointXYZRGB>);
       pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("r", pcl::ComparisonOps::LT, rMax)));
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("r", pcl::ComparisonOps::GT, rMin)));
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("g", pcl::ComparisonOps::LT, gMax)));
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("g", pcl::ComparisonOps::GT, gMin)));
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("b", pcl::ComparisonOps::LT, bMax)));
       color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
                                  (new pcl::PackedRGBComparison<pcl::PointXYZRGB>
                                   ("b", pcl::ComparisonOps::GT, bMin)));
        // build the filter
       pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
       condrem.setInputCloud (cloud_dist);
       cloudout = cloud_dist->makeShared();
       pcl::removeNaNFromPointCloud(*cloud_dist,*cloudout, indices);
       if(cloudout->points.size()<8)
           return 0;
       condrem.setKeepOrganized(true);
        // apply filter
       condrem.filter (*cloud_dist_color);
       cloudin = cloud_dist_color->makeShared();
       cloudout = cloud_dist_color->makeShared();
       return cloudin->points.size();
}






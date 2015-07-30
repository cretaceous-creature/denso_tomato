//jsk............chen

#include <pcproc.h>

TOMATODETECT::TOMATODETECT()
{
    Tracking_flag = 0;
}
TOMATODETECT::~TOMATODETECT()
{

}
//CloudIn
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
TOMATODETECT::CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string Param)
{
    //decode the param
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
   // sor.setInputCloud (cloud);
   // sor.setLeafSize (0.005f, 0.005f, 0.005f);
   // sor.filter (*cloud);
    cloudin = cloud;
    std::vector<int> indices;
    cloudout = cloud->makeShared();
    // pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    if(cloudout->points.size()<8)
        return cloudin->makeShared();

    PCsize = cloudin->points.size();
    istringstream in(Param);
    in>>RGBfilterL.r>>RGBfilterH.r
            >>RGBfilterL.g>>RGBfilterH.g
            >>RGBfilterL.b>>RGBfilterH.b
            >>Distfilter.min>>Distfilter.max
            >>Noisefilter.meank>>Noisefilter.Thresh;
    pcl::removeNaNFromPointCloud(*cloudin,*cloudin, indices);

    unsigned int pointnumafterRGBD
            = this->RGBDfilter();
    unsigned int pointnumafterNoise
            =this->NoiseFilter();
    return cloudout->makeShared();
}

unsigned int TOMATODETECT::RGBDfilter()
{

    // point cloud instance for the result
    // create passthrough filter instance
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
    pass_through.setFilterFieldName ("z");
    // set range for selected field to ~~~~~~ meters
    pass_through.setFilterLimits (Distfilter.min,Distfilter.max);
    // do filtering
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
    //condrem.filter (*cloud_dist_color);    //rgb filter


    //using hsi filter
    for(int i = 0; i<cloud_dist->size();i++){
        pcl::PointXYZRGB tmp_p;
        tmp_p = cloud_dist->points[i];
        HSI_SPACE tmp_hsi = Assign_HSI(tmp_p);
        if((tmp_hsi.H <= rMax && tmp_hsi.H >= rMin)||tmp_hsi.H >=rMin + 360 - rMax)
            if(tmp_hsi.S <= gMax && tmp_hsi.S >= gMin)
                if(tmp_hsi.I <= bMax && tmp_hsi.I >= bMin)
                    cloud_dist_color->push_back(tmp_p);

    }




    cloudin = cloud_dist_color->makeShared();
    cloudout = cloud_dist_color->makeShared();

    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    return cloudin->points.size();
}

HSI_SPACE TOMATODETECT::Assign_HSI(pcl::PointXYZRGB p)
{
    HSI_SPACE tmphsi;
    int RGBMAX,RGBMIN;
    RGBMAX = std::max(p.r,std::max(p.g,p.b));
    RGBMIN = std::min(p.r,std::min(p.g,p.b));
    tmphsi.I = (RGBMAX + RGBMIN)/2;

    if(RGBMAX == RGBMIN)
    {
        tmphsi.S = 0;
        tmphsi.H = 0;
    }
    else
    {
        if(tmphsi.I<128)
            tmphsi.S = 255 * (RGBMAX - RGBMIN)/(RGBMAX + RGBMIN);
        else
            tmphsi.S = 255 * (RGBMAX - RGBMIN)/(510 - (RGBMAX + RGBMIN));



        if(RGBMAX == p.r)
            tmphsi.H = 0 + 60*(p.g-p.b)/(RGBMAX-RGBMIN);
        else if(RGBMAX == p.g)
            tmphsi.H = 120 + 60*(p.b-p.r)/(RGBMAX-RGBMIN);
        else
            tmphsi.H = 240 + 60*(p.r-p.g)/(RGBMAX-RGBMIN);

        if(tmphsi.H < 0)
            tmphsi.H+=360;
    }
    return tmphsi;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
TOMATODETECT::HandCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudout1;
    cloudin1 = cloud->makeShared();
    cloudout1 = cloud->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout1,*cloudout1, indices);
    if(cloudout1->points.size()<8)
        return cloudin1->makeShared();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dist (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_through;
    // set input cloud
    pass_through.setInputCloud (cloudin1);
    cloudout1 = cloudin1->makeShared();
    pcl::removeNaNFromPointCloud(*cloudin1,*cloudout1, indices);
    if(cloudout1->points.size()<8)
        return cloudin1->makeShared();

    //z
    pass_through.setInputCloud (cloudout1);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (0.3,1.4);
    pass_through.filter (*cloud_dist);

    pcl::removeNaNFromPointCloud(*cloud_dist,*cloud_dist, indices);
    //this->Transformtozero();
    return cloud_dist->makeShared();
}



unsigned int TOMATODETECT::NoiseFilter()
{
    // set output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dist_color_noise (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudin);

    std::vector<int> indices;
    cloudout = cloudin->makeShared();
    pcl::removeNaNFromPointCloud(*cloudin,*cloudout, indices);
    if(cloudout->points.size()<8)
        return 0;

    sor.setMeanK (Noisefilter.meank);
    sor.setStddevMulThresh (Noisefilter.Thresh);
    sor.filter (*cloud_dist_color_noise);

    cloudout = cloud_dist_color_noise->makeShared();

    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    /*
 if(cloudout->points.size()<1)
 {
     pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
     // build the filter
     outrem.setInputCloud(cloudout);
     outrem.setRadiusSearch(0.3);
     outrem.setMinNeighborsInRadius (10);
     // apply filter
     outrem.filter (*cloudout);

 }
 */
    return cloudout->points.size();
}

//return the square of the distance
inline float DistToVector(float x,float y, float z,Eigen::Vector4f centroid)
{
    return ((x-centroid[0])*(x-centroid[0])+
            (y-centroid[1])*(y-centroid[1])+
            (z-centroid[2])*(z-centroid[2]));
}

/******************************************************
Get filtered point cloud center
using k-means when k=1;

*******************************************************/
int TOMATODETECT::GetCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    Eigen::Vector4f centroid;
    int index = -1;
    Center_vector.clear();
    Boundingbox.clear();
    float min = 100000;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
    if(cloud->points.size()<10)
        return -1;
    /*
    compute3DCentroid (*cloud, centroid);
    while(Removeoutliner(cloud, &centroid)>0.15*0.15){
        compute3DCentroid (*cloud, centroid);
    }
    //here we got the centroid after removing the outliner...

    printf("center= %f, %f, %f,\n",centroid[0],centroid[1],centroid[2]);
    */
    //using euclidean filter........

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    pcl::ModelCoefficients tmp_Coeff;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.08); // 6cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
         it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin ();
             pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        j++;
        pcl::PointXYZRGB P_min, P_max;
        compute3DCentroid (*cloud_cluster, centroid);

        pcl::getMinMax3D (*cloud_cluster, P_min, P_max);
        tmp_Coeff.values.resize(9);
        tmp_Coeff.values[0] = P_min.x - 0.02;  // x_min
        tmp_Coeff.values[1] = P_max.x + 0.02;  // x_max
        tmp_Coeff.values[2] = P_min.y - 0.02;  // y_min
        tmp_Coeff.values[3] = P_max.y + 0.02;  // y_max
        tmp_Coeff.values[4] = P_min.z - 0.02;  // z_min
        tmp_Coeff.values[5] = P_max.z + 0.02;  // z_max
        tmp_Coeff.values[6] = 0.0;  // r
        tmp_Coeff.values[7] = 1.0;  // g
        tmp_Coeff.values[8] = 0.0;  // b

        float kk = DistToVector(0,0,0,centroid);
        if(min > kk){
            min = kk;
            index = j-1;
        }
        if(abs(centroid[0])<1){
            Center_vector.push_back(centroid);
            Boundingbox.push_back(tmp_Coeff);
        }
    }
    // set the box parameter
    std::string guiparam;
    ros::param::get("/Clusternum",guiparam);
    istringstream in(guiparam);
    int orderindex;
    in >> orderindex;

    if(orderindex>0 && orderindex<=Boundingbox.size() && orderindex<=Center_vector.size()){
        index = orderindex - 1;
        Boundingbox[index].values[6] = 1.0;  // r
        Boundingbox[index].values[7] = 0.0;  // g
        LastCenter = Center_vector.at(index);
    }
    else
        index = -1;

    Tracking_flag = 1;
    return index;

}


float TOMATODETECT::Removeoutliner(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   Eigen::Vector4f *centroid)
{
    float max_dist_to_centroid=0;
    int index = 0;
    for(int i=0;i<cloud->points.size();i++)
    {
        float kk = DistToVector(cloud->points[i].x,cloud->points[i].y,
                                cloud->points[i].z,*centroid);
        if(Tracking_flag){
            kk += track_weight * DistToVector(cloud->points[i].x,cloud->points[i].y,
                                              cloud->points[i].z, LastCenter);
        }
        if(kk>max_dist_to_centroid){
            max_dist_to_centroid = kk;
            index = i;
        }
    }
    cloud->erase(cloud->points.begin()+index);
    //need to return the average distance
    return max_dist_to_centroid;
}






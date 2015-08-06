//jsk............chen

#include <seg_process.h>



TOMATOSEGMENTATION::TOMATOSEGMENTATION()
{
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
}
TOMATOSEGMENTATION::~TOMATOSEGMENTATION()
{
}
//CloudIn
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
TOMATOSEGMENTATION::CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string Param, std::string Parambox)
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
    float minx,miny,minz,maxx,maxy,maxz;
    istringstream in1(Parambox);
    in1>>minx>>maxx>>miny>>maxy>>minz>>maxz;

    Distfilter.min_x = minx;
    Distfilter.max_x = maxx;
    Distfilter.min_y = miny;
    Distfilter.max_y = maxy;
    Distfilter.min_z = minz;
    Distfilter.max_z = maxz;

    int size = this->Distancefilter();
    //this->Transformtozero();
    return cloudout->makeShared();
}


pcl::PointCloud<Pointhandeye>::Ptr
TOMATOSEGMENTATION::HandCloudFilter(pcl::PointCloud<Pointhandeye>::Ptr cloud)
{
    std::vector<int> indices;
    pcl::PointCloud<Pointhandeye>::Ptr cloudin1;
    pcl::PointCloud<Pointhandeye>::Ptr cloudout1;
    cloudin1 = cloud->makeShared();
    cloudout1 = cloud->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout1,*cloudout1, indices);
    if(cloudout1->points.size()<8)
        return cloudin1->makeShared();

    pcl::PointCloud<Pointhandeye>::Ptr cloud_dist (new pcl::PointCloud<Pointhandeye>);
    pcl::PassThrough<Pointhandeye> pass_through;
    // set input cloud
    pass_through.setInputCloud (cloudin1);
    cloudout1 = cloudin1->makeShared();
    pcl::removeNaNFromPointCloud(*cloudin1,*cloudout1, indices);
    if(cloudout1->points.size()<8)
        return cloudin1->makeShared();

    pass_through.setInputCloud (cloudout1);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (0.3,1.4);
    pass_through.filter (*cloud_dist);

    pcl::removeNaNFromPointCloud(*cloud_dist,*cloud_dist, indices);
    //this->Transformtozero();
    return cloud_dist->makeShared();
}

unsigned int TOMATOSEGMENTATION::Distancefilter()
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

    pcl::removeNaNFromPointCloud(*cloud_dist,*cloud_dist, indices);
    //then color filtering.,..........................................................................
    int rMax, rMin, gMax, gMin, bMax, bMin;
    std::string SegmentColor;
    ros::param::get("/colorseg",SegmentColor);
    istringstream in(SegmentColor);
    in >> rMin >> rMax >> gMin >> gMax >> bMin >> bMax;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dist_color (new pcl::PointCloud<pcl::PointXYZRGB>);

    /*
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
       */
    for(int i = 0; i<cloud_dist->size();i++){
        pcl::PointXYZRGB tmp_p;
        tmp_p = cloud_dist->points[i];
        if(tmp_p.r <= rMax && tmp_p.r >= 0)
            if(tmp_p.b <= 250 && tmp_p.b >= bMin)
                if(tmp_p.g <= 250 && tmp_p.g >= gMin)
                    cloud_dist_color->push_back(tmp_p);

    }

    cloudout = cloud_dist_color->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    return cloudout->points.size();
}

void TOMATOSEGMENTATION::Transformtozero()
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
TOMATOSEGMENTATION::ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                            pcl::PointCloud<Pointhandeye>::Ptr cloudframe)
{
    //if first time
    if(cloudall->points.size()==0)
        return cloudframe->makeShared();
    //ICP
    if(cloudframe->points.size()==0)
        return cloudall->makeShared();

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudall,*cloudall, indices);
    pcl::removeNaNFromPointCloud(*cloudframe,*cloudframe, indices);
    if(cloudall->points.size()==0)
        return cloudframe->makeShared();
    if(cloudframe->points.size()==0)
        return cloudall->makeShared();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, Pointhandeye> icp;
    icp.setInputCloud(cloudall);            //original point cloud
    icp.setInputTarget(cloudframe);         //the cloud to move to registrate to original one
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    if(icp.hasConverged()){
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        // pcl::transformPointCloud(*cloudframe->makeShared(), *cloudframe, transformation_matrix);//
        //Final = Final + *cloudframe;
        return Final.makeShared();
    }
    else
        return cloudall->makeShared();
}
pcl::PointCloud<PointT1>::Ptr
TOMATOSEGMENTATION::Polyfit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<PointT1> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, PointT1> mls;

    mls.setComputeNormals (true);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.01);

    // Reconstruct
    mls.process (mls_points);

    return mls_points.makeShared();
}

int TOMATOSEGMENTATION::Sphere_seg(pcl::PointCloud<PointT1>::Ptr cloud)
{
    // All the objects needed
    //clear the vector

    // Datasets
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_tomato (new pcl::ModelCoefficients);

    ros::param::get("/SSegment",SegmentParam);
    istringstream in(SegmentParam);
    float Rmin, Rmax, Dist_weight, Dist_thresh;
    in >> Rmin >> Rmax >> Dist_weight >> Dist_thresh;

    // Build a passthrough filter to remove spurious NaNs
    cloud_filtered = cloud->makeShared();

    int i = 0;
    int cloudpointsnum = cloud->points.size();
    int counter = 0;
    if(!cloud->empty())
        while(1){
            pcl::NormalEstimation<PointT, pcl::Normal> ne;
            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
            pcl::ExtractIndices<PointT> extract;
            pcl::ExtractIndices<pcl::Normal> extract_normals;
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            pcl::PointIndices::Ptr inliers_tomato (new pcl::PointIndices);
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_SPHERE);
            seg.setMethodType (pcl::SAC_PROSAC);
            seg.setNormalDistanceWeight (Dist_weight);  //this affect how many could be detected, like the fangchajuli....
            seg.setMaxIterations (10000);
            seg.setDistanceThreshold (Dist_thresh);   //searching length this directlly decide how many could be detected
            seg.setRadiusLimits (Rmin, Rmax);
            // Estimate point normals
            ne.setSearchMethod (tree);
            ne.setInputCloud (cloud_filtered);
            ne.setKSearch (50);
            ne.compute (*cloud_normals);
            seg.setInputCloud (cloud_filtered);
            seg.setInputNormals (cloud_normals);

            // Obtain the tomato inliers and coefficients
            if(cloud_filtered->size()<50)
                break;
            seg.segment (*inliers_tomato, *coefficients_tomato);
            //std::cerr << "tomato coefficients: " << *coefficients_tomato << std::endl;

            // Write the tomato inliers to disk
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_tomato);
            extract.setNegative (false);
            pcl::PointCloud<PointT>::Ptr cloud_tomato (new pcl::PointCloud<PointT> ());
            extract.filter (*cloud_tomato);

            if (cloud_tomato->points.empty ())
            {
                std::cerr << "Can't find the component." << std::endl;
                //return i;
                counter++;
                if(counter>1)
                    break;
                else
                    continue;
            }
            else
            {
                counter = 0;
                //std::stringstream ss;
                //ss << "figure" << i++ << ".pcd";
                //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_tomato, false);
                // Remove the tomato inliers, extract the rest
                //remove the detected tomato cloud from the whole cloud
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers_tomato);
                extract.setNegative (true);
                extract.filter (*cloud_filtered);
                extract_normals.setNegative (true);
                extract_normals.setInputCloud (cloud_normals);
                extract_normals.setIndices (inliers_tomato);
                extract_normals.filter (*cloud_normals);

                SegModels tmpmodel;
                tmpmodel.ModelCoeff = *coefficients_tomato;
                tmpmodel.Set_cloud = cloud_tomato->makeShared();
                //made an comparation that the distance to the center will be larger than the points, or it will
                //be wrong estimation....
                Eigen::Vector4f centroid_tomato;
                compute3DCentroid (*cloud_tomato, centroid_tomato);
                float distcentroid = centroid_tomato[0]*centroid_tomato[0]+
                        centroid_tomato[1]*centroid_tomato[1]+
                        centroid_tomato[2]*centroid_tomato[2];
                float distsphere = tmpmodel.ModelCoeff.values[0]*tmpmodel.ModelCoeff.values[0]+
                        tmpmodel.ModelCoeff.values[1]*tmpmodel.ModelCoeff.values[1]+
                        tmpmodel.ModelCoeff.values[2]*tmpmodel.ModelCoeff.values[2];

                if(cloud_tomato->points.size() < cloudpointsnum/25 || distcentroid > distsphere){ // 0.1 percent of points...
                    std::cerr << "Ignore figure in" << cloud_tomato->points.size () << " points." << std::endl;

                }
                else
                    Tomato_model.push_back(tmpmodel);


            }
        }
    return 0;

}


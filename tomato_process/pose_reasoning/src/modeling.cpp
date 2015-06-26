//jsk............chen

#include <modeling.h>



MODELING::MODELING()
{
    u = 0.3;
    sig = sqrt(5);
}
MODELING::~MODELING()
{
}
//CloudIn
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MODELING::CloudIN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //decode the param
    cloudin = cloud;
    std::vector<int> indices;
    cloudout = cloud->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    if(cloudout->points.size()<8)
        return cloudin->makeShared();


    //std::string gvector;
    //ros::param::get("/gvector",gvector);
    //istringstream is(gvector);
    //float x1,y1,z1,x2,y2,z2;
    //is >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
    //G_vector.x = x2 - x1;
   // G_vector.y = y2 - y1;
   // G_vector.z = z2 - z1;      //ofcourse we need to normalize it...
    compute3DCentroid (*cloudout, centroid);
   // Vector_Normalize(&G_vector);
    for(int i=0;i<Tomato_model.size();i++){
        Tomato_model[i].forcevector.clear();
        Tomato_model[i].BV.clear();
        ParamXYZ temp = G_vector;
        Vector_Normalize(&temp,0.1);
        Tomato_model[i].forcevector.push_back(temp);
    }

    //analyze for every tomato....
    Assignforce();

    //then we need to compute the boarder of the heading vector for each tomato
    // one force(gravity, only one boarder...  )
    // two force(two boarder...  one line....)
    // three force(four boarder... choose min and max..)
    //four force(8 boarder...   choose min and max...)
    //..............
    //.............
    //the first force vector is gravity
    Assignboarder();
    Assignpedicel_e();


    //int size = this->Distancefilter();
    //this->Transformtozero();
    return cloudout->makeShared();
}
float MODELING::ReturnGauss(float length)
{
    float left = 1/(sig*sqrt(2*PI));
    float right = pow(loge, -(length - u)*(length - u)/(2*sig*sig));
    return (left*right);
}

//return the radius of the up tomato and take that one as the weight
bool MODELING::Return_up(ParamXYZ vector)
{
    ParamXYZ end;
    end.x = vector.x/2 + G_vector.x;
    end.y = vector.y/2 + G_vector.y;
    end.z = vector.z/2 + G_vector.z;

    float dist1 = end.x*end.x + end.y*end.y + end.z*end.z;
    float dist2 = (end.x-vector.x)*(end.x-vector.x)+
            (end.x-vector.y)*(end.x-vector.y)+
            (end.x-vector.z)*(end.x-vector.z);
    if(dist1 > dist2)
        return true;
    else
        return false;

}

void MODELING::Assignforce()
{
    for(int i=0;i<Tomato_model.size()-1;i++){  //first to the end...
        for(int j=i+1;j<Tomato_model.size();j++){
            ParamXYZ tmpvector;
            tmpvector.x = Tomato_model[j].Modelcoeff.values[0] -
                    Tomato_model[i].Modelcoeff.values[0];     //back to first vector
            tmpvector.y = Tomato_model[j].Modelcoeff.values[1] -
                    Tomato_model[i].Modelcoeff.values[1];
            tmpvector.z = Tomato_model[j].Modelcoeff.values[2] -
                    Tomato_model[i].Modelcoeff.values[2];
            float spheredist = sqrt(tmpvector.x*tmpvector.x + tmpvector.y*tmpvector.y + tmpvector.z*tmpvector.z)
                    -(Tomato_model[i].Modelcoeff.values[3]+Tomato_model[j].Modelcoeff.values[3]);
            //compare with the length of the radius to judge if it is available...
            if(spheredist > 0.01) //larger than 1cm then we can assume that there is no connection
                continue;
            else{
                if(Return_up(tmpvector))
                    tmpvector.radius = Tomato_model[i].Modelcoeff.values[3];
                else
                    tmpvector.radius = Tomato_model[j].Modelcoeff.values[3];
                Vector_Normalize(&tmpvector,0.10);
                Tomato_model[j].forcevector.push_back(tmpvector);
                tmpvector.x = -tmpvector.x;
                tmpvector.y = -tmpvector.y;
                tmpvector.z = -tmpvector.z;
                Tomato_model[i].forcevector.push_back(tmpvector);
            }
        }
    }
}


void MODELING::Assignboarder()
{
    for(int i=0;i<Tomato_model.size();i++){
        for(int j=0;j<pow(2,Tomato_model[i].forcevector.size()-1);j++){
            BOARDER tmp_BD;
            // here is very difficult.......
            ParamXYZ tmp_vector = G_vector;
            float weight = 1.0;
            for(int kk=0;kk<Tomato_model[i].forcevector.size()-1;kk++){
                if((j>>kk) & 0x0001){
                    //multiply 10 because of top code
                    //normalize with length = 0.1
                    tmp_vector.x += (10 * Tomato_model[i].forcevector[kk+1].x);
                    tmp_vector.y += (10 * Tomato_model[i].forcevector[kk+1].y);
                    tmp_vector.z += (10 * Tomato_model[i].forcevector[kk+1].z);
                    tmp_vector.radius = Tomato_model[i].forcevector[kk+1].radius;
                    weight = 0.3;//ReturnGauss(1); //weight reduce.....
                }
            }
            tmp_BD.vector.x = -tmp_vector.x;
            tmp_BD.vector.y = -tmp_vector.y;
            tmp_BD.vector.z = -tmp_vector.z;
            tmp_BD.vector.radius = tmp_vector.radius;
            tmp_BD.weight = weight;
            Tomato_model[i].BV.push_back(tmp_BD);
        }
        Tomato_model[i].BV[0].weight = 1;   //first is gravity  set to one...
    }

}

void MODELING::Assignpedicel_e()
{
    BOARDER tmp_pedicel;
    for(int i=0;i<Tomato_model.size();i++){
        tmp_pedicel.vector.x = 0;
        tmp_pedicel.vector.y = 0;
        tmp_pedicel.vector.z = 0;
        for(int j=0;j<Tomato_model[i].BV.size();j++){
            tmp_pedicel.vector.x += (Tomato_model[i].BV[j].weight * Tomato_model[i].BV[j].vector.x);
            tmp_pedicel.vector.y += (Tomato_model[i].BV[j].weight * Tomato_model[i].BV[j].vector.y);
            tmp_pedicel.vector.z += (Tomato_model[i].BV[j].weight * Tomato_model[i].BV[j].vector.z);
        }
        Vector_Normalize(&tmp_pedicel.vector,1);
        Tomato_model[i].Pedicel_e = tmp_pedicel;
    }
}


unsigned int MODELING::Distancefilter()
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

    for(int i = 0; i<cloud_dist->size();i++){
        pcl::PointXYZRGB tmp_p;
        tmp_p = cloud_dist->points[i];
        if(tmp_p.r <= rMax && tmp_p.r >= rMin)
            if(tmp_p.b <= bMax && tmp_p.b >= bMin)
                if(tmp_p.g <= gMax && tmp_p.g >= gMin)
                    cloud_dist_color->push_back(tmp_p);

    }

    cloudout = cloud_dist_color->makeShared();
    pcl::removeNaNFromPointCloud(*cloudout,*cloudout, indices);
    return cloudout->points.size();
}

void MODELING::Vector_Normalize(ParamXYZ *vector,float length)
{
    float dist =  ((vector->x)*(vector->x)+(vector->y)*(vector->y)+(vector->z)*(vector->z));
    float nor_coef = sqrt((length*length)/dist);
    vector->x *= nor_coef;
    vector->y *= nor_coef;
    vector->z *= nor_coef;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MODELING::ICP_CON(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudall,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudframe)
{
    //if first time
    if(cloudall->points.size()==0)
        return cloudframe->makeShared();
    //ICP
    if(cloudframe->points.size()==0)
        return cloudall->makeShared();

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputCloud(cloudall);
    icp.setInputTarget(cloudframe);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    if(icp.hasConverged())
        return Final.makeShared();
    else
        return cloudall->makeShared();
}




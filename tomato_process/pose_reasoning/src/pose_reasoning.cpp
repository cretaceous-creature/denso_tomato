//jsk...........chen
/****************************
    reasoning for tomato pose

    make only one package :
   -DCATKIN_WHITELIST_PACKAGES="tomato_seg;rgbtocloud;pose_reasoning"
*****************************/
#include <pose_reasoning.h>
#include <tf/transform_broadcaster.h>
/*********************************************
        Get the Callback and visualize
*********************************************/
static boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
static std::string Paramcoeff, ParamBox, Paramlastcoeff;
static pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
static pcl::PointCloud<pcl::PointXYZRGB> cloud_all;
static pcl::PointCloud<pcl::PointXYZRGB> leap_cloud;
static char viewtype = 0;
static bool pub_flag = true;
MODELING *TEST = new MODELING;
#define cutting_dist 0.02


void tf_transmit(float x, float y, float z, std::string ss)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "camera_rgb_optical_frame" , ss));
}

/**************************
    Keyboard event
**************************/
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
{

    if(event.keyUp())
    {
        switch(event.getKeyCode())
        {
        case 'n':
        {
            viewtype++;
            if(viewtype>2)
                viewtype=0;
            break;
        }
        case 'z':
        {
            pub_flag = true;

            TEST->Tomato_model.clear();
            break;
        }
        case 'a':
        {
           pub_flag = false;
           break;
        }
            //send cutting information to euslisp
        case 'v':
        {
            //send to points.. one is the center of the tomato to pick
            //one is the point that the pedicel lies.....
            float x,y,z,r;
            if(TEST->Tomato_model.size()){
                r = TEST->Tomato_model[0].Modelcoeff.values[3];
                x = TEST->Tomato_model[0].Modelcoeff.values[0] - (r+cutting_dist) * TEST->G_vector.x;
                y = TEST->Tomato_model[0].Modelcoeff.values[1] - (r+cutting_dist) * TEST->G_vector.y;
                z = TEST->Tomato_model[0].Modelcoeff.values[2] - (r+cutting_dist) * TEST->G_vector.z;
                tf_transmit(x, y, z, "pcenter");
                if(TEST->Tomato_model[0].BV.size()>1){
                    x = TEST->Tomato_model[0].Modelcoeff.values[0] + (r+cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.x;
                    y = TEST->Tomato_model[0].Modelcoeff.values[1] + (r+cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.y;
                    z = TEST->Tomato_model[0].Modelcoeff.values[2] + (r+cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.z;
                }
                tf_transmit(x, y, z, "ppedicel");

            }

            break;
        }
        }
    }
}

//viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Modeling"));
    viewer->setBackgroundColor (1, 1, 1);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback(keyboardEvent);
    return (viewer);
}

static Eigen::Vector4f Center;
static std::string tomatoid[10]; //at most 10 sample
static ros::Publisher pub_pedicels;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.removeAllPointClouds();
    //add pc
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "Branch");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Branch");
    viewer.removeAllShapes();

    //draw the segmented tomatoes
    if(!viewtype)
        for(int i=0;i<(TEST->Tomato_model.size()>10?10:TEST->Tomato_model.size())
            ;i++){
            std::stringstream ss;
            ss << "tomato" << i;
            pcl::PointXYZ PC(TEST->Tomato_model[i].Modelcoeff.values[0],
                    TEST->Tomato_model[i].Modelcoeff.values[1],
                    TEST->Tomato_model[i].Modelcoeff.values[2]);
            viewer.addSphere(PC,TEST->Tomato_model[i].Modelcoeff.values[3],(1/(float)(i+1)),0,0,
                    ss.str());
            for(int j=0;j<TEST->Tomato_model[i].forcevector.size();j++){
                std::stringstream ss1;
                ss1 << "force" << j;
                float x1 = TEST->Tomato_model[i].Modelcoeff.values[0];
                float y1 = TEST->Tomato_model[i].Modelcoeff.values[1];
                float z1 = TEST->Tomato_model[i].Modelcoeff.values[2];
                float x2 = x1 + TEST->Tomato_model[i].forcevector[j].x;
                float y2 = y1 + TEST->Tomato_model[i].forcevector[j].y;
                float z2 = z1 + TEST->Tomato_model[i].forcevector[j].z;
                pcl::PointXYZ PG1(x1,y1,z1);
                pcl::PointXYZ PG2(x2,y2,z2);
                viewer.addArrow(PG2,PG1,i/5+0.4,1/(i+1),1/(i+1),false,ss.str()+ss1.str());
            }
        }
    else if(viewtype == 1)
        // show boarder vector
        for(int i=0;i<(TEST->Tomato_model.size()>10?10:TEST->Tomato_model.size())
            ;i++){
            std::stringstream ss;
            ss << "tomato" << i;
            pcl::PointXYZ PC(TEST->Tomato_model[i].Modelcoeff.values[0],
                    TEST->Tomato_model[i].Modelcoeff.values[1],
                    TEST->Tomato_model[i].Modelcoeff.values[2]);
          //  viewer.addSphere(PC,TEST->Tomato_model[i].Modelcoeff.values[3],(1/(float)(i+1)),0,0,
           //         ss.str());
            for(int j=0;j<TEST->Tomato_model[i].BV.size();j++){
                std::stringstream ss1;
                ss1 << "force" << j;
                float x1 = TEST->Tomato_model[i].Modelcoeff.values[0];
                float y1 = TEST->Tomato_model[i].Modelcoeff.values[1];
                float z1 = TEST->Tomato_model[i].Modelcoeff.values[2];
                float x2 = x1 + 0.1*TEST->Tomato_model[i].BV[j].vector.x;
                float y2 = y1 + 0.1*TEST->Tomato_model[i].BV[j].vector.y;
                float z2 = z1 + 0.1*TEST->Tomato_model[i].BV[j].vector.z;
                pcl::PointXYZ PG1(x1,y1,z1);
                pcl::PointXYZ PG2(x2,y2,z2);
            //    viewer.addArrow(PG1,PG2,i/5+0.4,1/(i+1),false,0,ss.str()+ss1.str());
            }
        }
    else
        for(int i=0;i<(TEST->Tomato_model.size()>10?10:TEST->Tomato_model.size())
            ;i++){
            std::stringstream ss;
            ss << "tomato" << i;
            pcl::PointXYZ PC(TEST->Tomato_model[i].Modelcoeff.values[0],
                    TEST->Tomato_model[i].Modelcoeff.values[1],
                    TEST->Tomato_model[i].Modelcoeff.values[2]);
            viewer.addSphere(PC,TEST->Tomato_model[i].Modelcoeff.values[3],(1/(float)(i+1)),0,0,
                    ss.str());

            std::stringstream ss1,ss2;
            ss1 << "force1";
            ss2 << "force2";
            float x1 = TEST->Tomato_model[i].Modelcoeff.values[0];
            float y1 = TEST->Tomato_model[i].Modelcoeff.values[1];
            float z1 = TEST->Tomato_model[i].Modelcoeff.values[2];
            float x2 = x1 + 0.1*TEST->Tomato_model[i].BV[0].vector.x;
            float y2 = y1 + 0.1*TEST->Tomato_model[i].BV[0].vector.y;
            float z2 = z1 + 0.1*TEST->Tomato_model[i].BV[0].vector.z;
            //pedicel direction
            float x3 = x1 + 0.1*TEST->Tomato_model[i].Pedicel_e.vector.x;
            float y3 = y1 + 0.1*TEST->Tomato_model[i].Pedicel_e.vector.y;
            float z3 = z1 + 0.1*TEST->Tomato_model[i].Pedicel_e.vector.z;
            pcl::PointXYZ PG1(x1,y1,z1);
            pcl::PointXYZ PG2(x2,y2,z2);
            pcl::PointXYZ PG3(x3,y3,z3);
            //viewer.addArrow(PG1,PG2,i/5+0.4,1/(i+1),1/(i+1),1,1/(i+1),0,ss.str()+ss1.str());
            //viewer.addArrow(PG1,PG3,i/5+0.4,1/(i+1),1/(i+1),1,1/(i+1),0,ss.str()+ss2.str());
            viewer.addArrow(PG2,PG1,i/5+0.4,1/(i+1),1/(i+1),false,ss.str()+ss1.str());
            viewer.addArrow(PG3,PG1,i/5+0.4,1/(i+1),1/(i+1),false,ss.str()+ss2.str());

        }

    // broadcast TF
    {
      float x,y,z,r;
      if(TEST->Tomato_model.size()){
        r = TEST->Tomato_model[0].Modelcoeff.values[3];
        x = TEST->Tomato_model[0].Modelcoeff.values[0] -
            (r + cutting_dist) * TEST->G_vector.x;
        y = TEST->Tomato_model[0].Modelcoeff.values[1] -
            (r + cutting_dist) * TEST->G_vector.y;
        z = TEST->Tomato_model[0].Modelcoeff.values[2] -
            (r + cutting_dist) * TEST->G_vector.z;
        tf_transmit(x, y, z, "pcenter");
        if(TEST->Tomato_model[0].BV.size() > 1){
          x = TEST->Tomato_model[0].Modelcoeff.values[0] +
              (r + cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.x;
          y = TEST->Tomato_model[0].Modelcoeff.values[1] +
              (r + cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.y;
          z = TEST->Tomato_model[0].Modelcoeff.values[2] +
              (r + cutting_dist) * TEST->Tomato_model[0].Pedicel_e.vector.z;
        }
        tf_transmit(x, y, z, "ppedicel");
      }
    }  // TF

    // publish marker
    visualization_msgs::Marker pedicels_marker;
    pedicels_marker.header.frame_id = "/camera_rgb_optical_frame";
    pedicels_marker.header.stamp = ros::Time::now();
    pedicels_marker.ns = "tomato_pedicels";
    pedicels_marker.id = 0;
    pedicels_marker.type = visualization_msgs::Marker::LINE_LIST;
    pedicels_marker.action = visualization_msgs::Marker::ADD;
    pedicels_marker.pose.position.x = 0.0;
    pedicels_marker.pose.position.y = 0.0;
    pedicels_marker.pose.position.z = 0.0;
    pedicels_marker.pose.orientation.x = 0.0;
    pedicels_marker.pose.orientation.y = 0.0;
    pedicels_marker.pose.orientation.z = 0.0;
    pedicels_marker.pose.orientation.w = 1.0;
    pedicels_marker.scale.x = 0.005;
    pedicels_marker.scale.y = 1.0;
    pedicels_marker.scale.z = 1.0;
    pedicels_marker.color.a = 0.5;
    pedicels_marker.color.r = 0.0;
    pedicels_marker.color.g = 1.0;
    pedicels_marker.color.b = 0.0;

    for(int i = 0; i < (TEST->Tomato_model.size() > 10 ? 10 : TEST->Tomato_model.size()); i++) {
      if (TEST->Tomato_model[0].BV.size() > 1) {
        double r = TEST->Tomato_model[i].Modelcoeff.values[3];
        double x1 = TEST->Tomato_model[i].Modelcoeff.values[0];
        double x2 = x1 +
            (r + cutting_dist) * TEST->Tomato_model[i].Pedicel_e.vector.x;
        double y1 = TEST->Tomato_model[i].Modelcoeff.values[1];
        double y2 = y1 +
            (r + cutting_dist) * TEST->Tomato_model[i].Pedicel_e.vector.y;
        double z1 = TEST->Tomato_model[i].Modelcoeff.values[2];
        double z2 = z1 +
            (r + cutting_dist) * TEST->Tomato_model[i].Pedicel_e.vector.z;

        geometry_msgs::Point geom_point1;
        geom_point1.x = x1;
        geom_point1.y = y1;
        geom_point1.z = z1;
        geometry_msgs::Point geom_point2;
        geom_point2.x = x2;
        geom_point2.y = y2;
        geom_point2.z = z2;
        pedicels_marker.points.push_back(geom_point1);
        pedicels_marker.points.push_back(geom_point2);    
      }
    }

    pub_pedicels.publish(pedicels_marker);

    //draw box
    std::stringstream ss;
    ss << "branch";
    ros::param::get("/Boxsize",ParamBox);
    float minx,miny,minz,maxx,maxy,maxz;
    istringstream in(ParamBox);
    in>>minx>>maxx>>miny>>maxy>>minz>>maxz;
    //draw gravity
    pcl::PointXYZ PG1((minx+maxx)/2,maxy,maxz);
    pcl::PointXYZ PG2((minx+maxx)/2+ 0.1 *TEST->G_vector.x,maxy+0.1 *TEST->G_vector.y,maxz+0.1 *TEST->G_vector.z);
   // viewer.addArrow(PG2,PG1,0,0,1,false,"gravity");
    viewer.addCube(
                minx,
                maxx,
                miny,
                maxy,
                minz,
                maxz,
                1.0,
                0.0,
                0.0,
                ss.str());

}
static int counter = 0;
#define process_freque 6

void LeapCloud(const sensor_msgs::PointCloud2 msg)
{
    // then convert the pc2 format to classic pcl::pc format
    pcl::fromROSMsg(msg,leap_cloud);
    viewerOneOff(*PCLviewer,leap_cloud.makeShared());
}

inline float Model_dist(MBT M1, MBT M2)
{
    return sqrt( (M1.Modelcoeff.values[0]-M2.Modelcoeff.values[0])*(M1.Modelcoeff.values[0]-M2.Modelcoeff.values[0])+
            (M1.Modelcoeff.values[1]-M2.Modelcoeff.values[1])*(M1.Modelcoeff.values[1]-M2.Modelcoeff.values[1])+
            (M1.Modelcoeff.values[2]-M2.Modelcoeff.values[2])*(M1.Modelcoeff.values[2]-M2.Modelcoeff.values[2]));
}

//call back function
void CloudCallBack(const sensor_msgs::PointCloud2 msg)
{
   if(pub_flag){
    ros::param::get("/tomatocoeff",Paramcoeff);
    istringstream in(Paramcoeff);

    std::string gvector;
    ros::param::get("/gvector",gvector);
    istringstream is(gvector);
    float x1,y1,z1,x2,y2,z2;
    is >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
    TEST->G_vector.x = x2 - x1;
    TEST->G_vector.y = y2 - y1;
    TEST->G_vector.z = z2 - z1;      //ofcourse we need to normalize it...
    TEST->Vector_Normalize(&TEST->G_vector);

    if(Paramcoeff.compare(Paramlastcoeff)){
        int modelnum;
        float x,y,z,radius;
        in>>modelnum; // num of models
        MBT ref_point;
        ref_point.Modelcoeff.values.push_back(- TEST->G_vector.x);  //little offset...
        ref_point.Modelcoeff.values.push_back(- TEST->G_vector.y);
        ref_point.Modelcoeff.values.push_back(- TEST->G_vector.z - 1.0);      // make this a vector with picking order...
        for(int i=0;i<modelnum;i++){
            ModelByTrack tmpcoeff;
            tmpcoeff.Appeartimes = 1;
            in>>x>>y>>z>>radius;
            tmpcoeff.Modelcoeff.values.push_back(x);
            tmpcoeff.Modelcoeff.values.push_back(y);
            tmpcoeff.Modelcoeff.values.push_back(z);
            tmpcoeff.Modelcoeff.values.push_back(radius);
            tmpcoeff.orderdist = Model_dist(ref_point,tmpcoeff);
            if(TEST->Tomato_model.empty()){
                TEST->Tomato_model.push_back(tmpcoeff);
            }
            else{
                for(int j=0;j<TEST->Tomato_model.size();j++){   //here try to combine tomatoes in fact in same position but detected as two due to the calibration, err?
                    //this condition should be carefully chosen, or it may suffer wrong detection....
                    if(tmpcoeff.Modelcoeff.values[3]+
                            TEST->Tomato_model[j].Modelcoeff.values[3]-        //r1 + r2 - dist(c1,c2)
                            Model_dist(tmpcoeff,TEST->Tomato_model[j])>0.025){
                        TEST->Tomato_model[j].Modelcoeff.values[0] = (TEST->Tomato_model[j].Modelcoeff.values[0]+tmpcoeff.Modelcoeff.values[0])/2;
                        TEST->Tomato_model[j].Modelcoeff.values[1] = (TEST->Tomato_model[j].Modelcoeff.values[1]+tmpcoeff.Modelcoeff.values[1])/2;
                        TEST->Tomato_model[j].Modelcoeff.values[2] = (TEST->Tomato_model[j].Modelcoeff.values[2]+tmpcoeff.Modelcoeff.values[2])/2;
                        TEST->Tomato_model[j].Modelcoeff.values[3] = (TEST->Tomato_model[j].Modelcoeff.values[3]+tmpcoeff.Modelcoeff.values[3])/2;
                        TEST->Tomato_model[j].Appeartimes += tmpcoeff.Appeartimes;
                        break;
                    }
                    //last time with out match
                    if(j == TEST->Tomato_model.size()-1){
                        for(int jj = 0; jj<TEST->Tomato_model.size(); jj++){
                            if(tmpcoeff.orderdist < TEST->Tomato_model[jj].orderdist){
                                TEST->Tomato_model.insert(TEST->Tomato_model.begin()+jj,
                                                          tmpcoeff);  //push in front of it...
                                break;
                            }
                            if(jj == TEST->Tomato_model.size()-1){
                                TEST->Tomato_model.push_back(tmpcoeff); //if is the fathest
                                break;
                            }
                        }
                        break;
                    }
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        // then convert the pc2 format to classic pcl::pc format
        pcl::fromROSMsg(msg,cloud);
        //deal with tomato
        cloud_all = *cloud.makeShared();
        //viewerOneOff(*PCLviewer,cloud_all.makeShared());
        if(counter++>process_freque){
            counter = 0;
            for(std::vector<ModelByTrack>::iterator it = TEST->Tomato_model.begin();it!=TEST->Tomato_model.end();++it){
                if(it->Appeartimes <= process_freque/3){
                    TEST->Tomato_model.erase(it);
                    it--;
                    continue;
                }
                it->Appeartimes = 0;
            }
            //here we remove the wrong detection then can do reasoning part....
            //we can build more node so that we could be quilker...
            cloud_frame = *(TEST->CloudIN(cloud.makeShared())->makeShared());
            viewerOneOff(*PCLviewer,cloud_all.makeShared());
        }
    }
    Paramlastcoeff = Paramcoeff;
}
}




int main(int argc, char **argv)
{
    ros::init(argc, argv,"reason_pose");
    ros::NodeHandle n;
    PCLviewer = rgbVis();

    pub_pedicels =
        n.advertise<visualization_msgs::Marker>("/tomato_pedicels", 1);

    //1000 equals to the message queue
    //dont be too large or it will become slow
    ros::Subscriber sub = n.subscribe("branchcloud",1,CloudCallBack);
    //ros::Subscriber sub1  = n.subscribe("/leap_motion/points2",1,LeapCloud);

    while (!PCLviewer->wasStopped ())
    {

        ros::spinOnce();
        PCLviewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
    }

    return 0;
}


//jsk...........chen
/****************************
    Trying to detect the plane that segmentate tomatoes

    make only one package :
   -DCATKIN_WHITELIST_PACKAGES="tomato_seg;rgbtocloud"
*****************************/
#include <tomato_seg.h>

/*********************************************
        Get the Callback and visualize
*********************************************/
static boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
static std::string ParamCentroid,Paramlast,ParamBox, ParamSegment;
//THIS FUNCTION WILL BE CALLED AT EVERY FRAME.   ABOUT 50MS?
static pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
static pcl::PointCloud<pcl::PointXYZRGB> cloud_all;
static pcl::PointCloud<pcl::PointXYZRGB> cloud_lastframe;
static pcl::PointCloud<Pointhandeye> Handeye_cloud;       //in fact its not RGB cloud
TOMATOSEGMENTATION *TEST = new TOMATOSEGMENTATION;
//viewer
static bool pub_flag = true;
/**************************
    Keyboard event
**************************/
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
{

    if(event.keyUp())
    {

        switch(event.getKeyCode())
        {
        //send cutting information to euslisp
        case 'n':
        {
            pub_flag = false;

        }
        case 'v':
        {
            pub_flag = true;

        }

            break;
        }
    }
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Object viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback(keyboardEvent);
    return (viewer);
}
static Eigen::Vector4f Center;
static std::string tomatoid[10]; //at most 10 sample
static ros::Publisher pub_sphere;
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.removeAllPointClouds();
    //add pc
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.removeAllShapes();


    // publish sphere
    visualization_msgs::MarkerArray fruits_markers;
    ros::Time tm_now = ros::Time::now();

    ostringstream os;
    os<<TEST->Tomato_model.size();

    //draw the segmented tomatoes
    for(int i=0;i<(TEST->Tomato_model.size()>10?10:TEST->Tomato_model.size())
        ;i++){
        std::stringstream ss;
        ss << "tomato" << i;
        viewer.addSphere(TEST->Tomato_model[i].ModelCoeff,
                         ss.str());
        float xt,yt,zt,radius;
        xt = TEST->Tomato_model[i].ModelCoeff.values[0];
        yt = TEST->Tomato_model[i].ModelCoeff.values[1];
        zt = TEST->Tomato_model[i].ModelCoeff.values[2];
        radius = TEST->Tomato_model[i].ModelCoeff.values[3];
        os<<" "<<xt<<" "<<yt<<" "<<zt<<" "<<radius;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/camera_rgb_optical_frame";
        marker.header.stamp = tm_now;
        marker.ns = "tomato_fruites";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = xt;
        marker.pose.position.y = yt;
        marker.pose.position.z = zt; 
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = radius * 2.0;
        marker.scale.y = radius * 2.0;
        marker.scale.z = radius * 2.0;
        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        fruits_markers.markers.push_back(marker);
    }
    std::string paramset(os.str());
    ros::param::set("/tomatocoeff",paramset);
    pub_sphere.publish(fruits_markers);

    //draw box
    std::stringstream ss;
    ss << "branch";
    float minx,miny,minz,maxx,maxy,maxz;
    istringstream in(ParamBox);
    in>>minx>>maxx>>miny>>maxy>>minz>>maxz;
    viewer.addCube(
                minx,
                maxx,
                miny,
                maxy,
                minz,
                maxz,
                1.0,
                1.0,
                1.0,
                ss.str());
}
tf::Transform HandeyetoCamera = tf::Transform::getIdentity();
void HandEye(const sensor_msgs::PointCloud2 msg)
{
    // then convert the pc2 format to classic pcl::pc format
    static pcl::PointCloud<Pointhandeye> Handeye_tmp;
    pcl::fromROSMsg(msg,Handeye_tmp);
    Handeye_tmp = *(TEST->HandCloudFilter(Handeye_tmp.makeShared()))->makeShared();
    pcl_ros::transformPointCloud(*Handeye_tmp.makeShared(),Handeye_tmp,HandeyetoCamera);
    Handeye_cloud += Handeye_tmp;
    // viewerOneOff(*PCLviewer,Handeye_cloud.makeShared());
}


//call back function
void CloudCallBack(const sensor_msgs::PointCloud2 msg)
{
    ros::param::get("/Centroid",ParamCentroid);
    ros::param::get("/Boxsize",ParamBox);
    //printf ("Param= %s \n %s \n", ParamCentroid.c_str(),ParamBox.c_str());
    if(ParamCentroid.compare(Paramlast))
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(msg,cloud);
        //deal with tomato
        //two methods, one is directly plus two point clouds,

        //cloud += Handeye_cloud;

        //the other one is registrating two point clouds using ICP
       // cloud = *(TEST->ICP_CON(cloud.makeShared(),
        //                        Handeye_cloud.makeShared()))->makeShared();


        cloud_frame = *(TEST->CloudIN(cloud.makeShared(),
                                      ParamCentroid,ParamBox))->makeShared();



        cloud_all = *(TEST->ICP_CON(cloud_lastframe.makeShared(),
                                   cloud_frame.makeShared()))->makeShared();

       // cloud_all = *cloud_frame.makeShared(),

        cloud_lastframe = *cloud_frame.makeShared();

        cloud_frame = *(TEST->CloudIN(Handeye_cloud.makeShared(),
                                      ParamCentroid,ParamBox))->makeShared();
        if(!cloud_all.points.empty()){
            //first polyfit then do segmentation
            pcl::PointCloud<PointT1> cloud_save;
            cloud_all = *(TEST->Polyfit(cloud_all.makeShared()))->makeShared();

            cloud_save = *cloud_all.makeShared();
            TEST->Tomato_model.clear();
            int num_total = TEST->Sphere_seg(cloud_save.makeShared());
            num_total = TEST->Sphere_seg(cloud_frame.makeShared());
        }
        viewerOneOff(*PCLviewer,cloud_all.makeShared());
        Handeye_cloud.clear();
    }
    Paramlast = ParamCentroid;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"icp_recon");
    ros::NodeHandle n;
    ros::Rate r(3);       //this control the publish frequency of cloud_msg
    // set initial parameters
    //  2 param     center and box
    ros::param::param<std::string>("/Centroid",ParamCentroid,
                                   "0.0 0.0 0.0");   //0,0,0
    ros::param::set("/Centroid","0.0 0.0 0.0");

    //ros::param::param<std::string>("/SSegment",ParamSegment,
     //                              "0.018 0.03 0.04 0.003");   //0,0,0
    //ros::param::set("/SSegment","0.018 0.03 0.04 0.002");

    ros::param::param<std::string>("/SSegment",ParamSegment,
                                  "0.03 0.04 0.04 0.0015");   //0,0,0
    ros::param::set("/SSegment","0.03 0.04 0.04 0.0010");

    ros::param::set("/colorseg","90 255 0 80 0 80");

    ros::param::param<std::string>("/Boxsize",ParamBox,
                                   "0.0 0.0 0.0 0.0 0.0 0.0");   //initial
    PCLviewer = rgbVis();
    //1000 equals to the message queue
    //dont be too large or it will become slow

    pub_sphere =
        n.advertise<visualization_msgs::MarkerArray>("/tomato_fruits", 1);

    ros::Subscriber sub = n.subscribe("/camera_remote/depth_registered/points",1,CloudCallBack);
    ros::Subscriber sub1  = n.subscribe("/camera1_remote/depth_registered/points",1,HandEye);

   // ros::Subscriber sub = n.subscribe("/camera_remote_uncalibrated/depth_registered/points",1,CloudCallBack);
   // ros::Subscriber sub1  = n.subscribe("/camera1/depth_registered/points",1,HandEye);

    sensor_msgs::PointCloud2 cloud_msg;
    std::string topic = n.resolveName("branchcloud");
    uint32_t queue_size = 1;
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topic, queue_size);

    tf::TransformListener listener;
    while (!PCLviewer->wasStopped ())
    {
        tf::StampedTransform transform;
        pcl::toROSMsg(*cloud_all.makeShared(),cloud_msg);
        cloud_msg.header.frame_id = "/camera_depth_optical_frame";
        if(pub_flag)
            pub.publish(cloud_msg);

//        try{
//            listener.lookupTransform(
//                        "/camera_rgb_optical_frame",
//                        "/cxy/camera1_rgb_optical_frame",
//                        ros::Time(0), transform);
//        }
//        catch (tf::TransformException &ex) {
//            ros::Duration(0.1).sleep();
//            continue;
//        }
        HandeyetoCamera = transform * tf::Transform::getIdentity();
        ros::spinOnce();
        r.sleep();
        PCLviewer->spinOnce (200);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
   // pcl::PointCloud<PointT1> cloud_save;
   // cloud_save = *(TEST->Polyfit(cloud_all.makeShared()))->makeShared();
    //int num_total = TEST->Sphere_seg(cloud_save.makeShared());
    //pcl::io::savePCDFileASCII ("PointcloudafterICP.pcd",  cloud_save);
    return 0;
}

//jsk...........chen
/****************************
 * For debug u need to run -DCMAKE_BUILD_TYPE=Debug   in CMake
 * *****************************/
#include <listenopenni.h>
#include <tf/transform_broadcaster.h>

/*********************************************
        Get the Callback and visualize
*********************************************/
//static pcl::visualization::CloudViewer CLOUDviewer ("SIMPLE CLOUD VIEWER");
static boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
static std::string ParamAll;
TOMATODETECT *TEST=new TOMATODETECT;
//THIS FUNCTION WILL BE CALLED AT EVERY FRAME.   ABOUT 50MS?
static pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
static pcl::PointCloud<pcl::PointXYZRGB> Handeye_cloud;
static int counter;


void tf_transmit(float x, float y, float z)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "camera_rgb_optical_frame" , "tomato"));
}
/**************************
    Keyboard event
**************************/
static bool viewchange = false;
static int bn = 1;
static Eigen::Vector4f Aim_branch;
static bool picking_flag = false;
static bool box_switch =true;
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
{

    if(event.keyUp())
    {
        switch(event.getKeyCode())
        {
        case 'z':{
            //change bounding box....
            ostringstream ss;
            bn = (bn<TEST->Center_vector.size()?bn+1:1);
            ss << bn;
            std::string paramset(ss.str());
            ros::param::set("/Clusternum",paramset);
            break;
        }
        case 'n':{
            //change bounding box....
            viewchange = !viewchange;
            picking_flag = false;
            break;
        }
        case 'j':{
            //change bounding box....
            box_switch = !box_switch;
            break;
        }
        case 'v':{
            //execute and move handeye...

            Aim_branch = TEST->Center_vector.at(bn-1);
            picking_flag = true;
            tf_transmit(Aim_branch[0], Aim_branch[1], Aim_branch[2]);
            //then publish something to move the robot...

            break;
        }
        }
    }
}



//viewer

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback(keyboardEvent);
    return (viewer);
}
static Eigen::Vector4f Center,Center_last;
static int initflag = 0;
static int Tracktime = 0;
inline float DistToCenter(Eigen::Vector4f centroid,Eigen::Vector4f centroid_last)
{
    return ((centroid[0]-centroid_last[0])*(centroid[0]-centroid_last[0])+
            (centroid[1]-centroid_last[1])*(centroid[1]-centroid_last[1])+
            (centroid[2]-centroid_last[2])*(centroid[2]-centroid_last[2]));
}
void easytracking()
{
    if(!initflag){
        initflag = 1;
        Center_last = Center;
        return;
    }
    //if > 1m  then means drift                        disttmping
    if(DistToCenter(Center,Center_last) > (TrackDist + 0.05 * Tracktime) &&
            !(Center_last[0]==0 && Center_last[1]==0 && Center_last[2]==0)){
        Center = Center_last;
        Tracktime++;
    }
    else
    {
        Center_last = Center;
        Tracktime = 0;
    }

}
#define framecount 2
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    easytracking();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.removeAllPointClouds();
    const std::string cylinderid = "cylinder";
    const std::string lineidz = "LINEZ";
    const std::string lineidy = "LINEY";
    const std::string lineidx = "LINEX";
    viewer.setBackgroundColor (1, 1, 1);
    viewer.removeShape(cylinderid);
    viewer.removeShape(lineidz);
    viewer.removeShape(lineidy);
    viewer.removeShape(lineidx);
    viewer.removeAllShapes();
    //add pc
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //add cylinder
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize (7);    // We need 7 values
    cylinder_coeff.values[0] = Center[0];
    cylinder_coeff.values[1] = Center[1]+0.05;
    cylinder_coeff.values[2] = Center[2]+0.01;
    cylinder_coeff.values[3] = 0;
    cylinder_coeff.values[4] = -0.1;
    cylinder_coeff.values[5] = 0;
    cylinder_coeff.values[6] = 0.05;
    //viewer.addCylinder(cylinder_coeff);

    // viewer.addClinder(Center[0]-0.05,Center[0]+0.05,
    //           Center[1]-0.05,Center[1]+0.05,
    //           Center[2]-0.04,Center[2]+0.06);

    pcl::PointXYZ P1(Center[0],Center[1],Center[2]+0.01);
    pcl::PointXYZ P2(Center[0],Center[1],Center[2]+0.11);
    pcl::PointXYZ P3(Center[0],Center[1]+0.1,Center[2]);
    pcl::PointXYZ P4(Center[0]+0.1,Center[1],Center[2]);

    //viewer.addLine<pcl::PointXYZ,  pcl::PointXYZ>(P1,P2);
    viewer.addLine(P1,P2,0,0,1,lineidz);
    viewer.addLine(P1,P3,0,1,0,lineidy);
    viewer.addLine(P1,P4,1,0,0,lineidx);
    ostringstream ss;
    float z = Center[2];
    float y = Center[1];
    float x = Center[0];
    ss<<x<<" "<<y<<" "<<z;
    std::string paramset(ss.str());
    ros::param::set("/Centroid",paramset);

    for(int i=0;i<TEST->Center_vector.size();i++){
        std::stringstream ss;
        ss << "branch" << i;
        pcl::ModelCoefficients cube_coeff;
        if(TEST->Boundingbox.size()>i){
            cube_coeff = TEST->Boundingbox[i];
            viewer.addCube(
                        cube_coeff.values[0],
                    cube_coeff.values[1],
                    cube_coeff.values[2],
                    cube_coeff.values[3],
                    cube_coeff.values[4],
                    cube_coeff.values[5],
                    cube_coeff.values[6],
                    cube_coeff.values[7],
                    cube_coeff.values[8],
                    ss.str());
        }
    }

    //draw the gravity direction
    std::string gvector;
    ros::param::get("/gvector",gvector);
    istringstream is(gvector);
    float x1,y1,z1,x2,y2,z2;
    is >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
    pcl::PointXYZ PG1(x1,y1,z1);
    pcl::PointXYZ PG2(x2,y2,z2);
    viewer.addArrow(PG2,PG1,0,0,1,false);


    //send the tf
    /*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame" , "tomato"));
    */


}
//hand eye
tf::Transform HandeyetoCamera = tf::Transform::getIdentity();
void HandEye(const sensor_msgs::PointCloud2 msg)
{
    // then convert the pc2 format to classic pcl::pc format
    pcl::fromROSMsg(msg,Handeye_cloud);
    Handeye_cloud = *(TEST->HandCloudFilter(Handeye_cloud.makeShared()))->makeShared();
    pcl_ros::transformPointCloud(*Handeye_cloud.makeShared(),Handeye_cloud,HandeyetoCamera);

    //leap_cloud.header.frame_id =
    //viewerOneOff(*PCLviewer,Handeye_cloud.makeShared());
}

//call back function
static int frame_divider = 0;
void CloudCallBack(const sensor_msgs::PointCloud2 msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(msg,cloud);

    //get parameters and then trying to applying filter
    printf ("Param= %s \n", ParamAll.c_str());
    ros::param::get("/MyParam",ParamAll);
    //then, process the cloud;
  //  if(counter++>framecount){
        counter=0;
        //show only the apple
        //cloud +=Handeye_cloud;
        cloud_frame = *(TEST->CloudIN(cloud.makeShared(),ParamAll))->makeShared();
        if(cloud_frame.points.size()>2){
            int index = TEST->GetCenter(cloud_frame.makeShared());
            //int index = -1; //for test
            if(index==-1){
                std::stringstream ss;
                ss<< 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0;
                std::string paramset(ss.str());
                ros::param::set("/Boxsize",paramset);
            }
            else{
                if(picking_flag){
                    float disttmp = 10000;
                    for(int i = 0;i<TEST->Center_vector.size();i++){
                        float tmp = sqrt(DistToCenter(TEST->Center_vector.at(i),Aim_branch));
                        if(tmp<disttmp){
                            disttmp = tmp;
                            index = i;
                        }
                    }
                }
                Center = TEST->Center_vector.at(index);

                std::stringstream ss;
                ss<< TEST->Boundingbox[index].values[0]<<" "<<
                                                         TEST->Boundingbox[index].values[1]<<" "<<
                                                         TEST->Boundingbox[index].values[2]<<" "<<
                                                         TEST->Boundingbox[index].values[3]<<" "<<
                                                         TEST->Boundingbox[index].values[4]<<" "<<
                                                         TEST->Boundingbox[index].values[5];
                std::string paramset(ss.str());
                if(box_switch)
                    ros::param::set("/Boxsize",paramset);
            }
            //now we got the center
            //CLOUDviewer.runOnVisualizationThreadOnce(viewerOneOff);
        }
        cloud_frame = *(TEST->CloudIN(cloud.makeShared(),ParamAll))->makeShared();
        //CLOUDviewer.showCloud(cloud.makeShared());
        if(viewchange)
            viewerOneOff(*PCLviewer,cloud_frame.makeShared());
        else
            viewerOneOff(*PCLviewer,cloud.makeShared());

   // }
    //else
    //    cloud_frame += *(TEST->CloudIN(cloud.makeShared(),ParamAll))->makeShared();



}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"listenopenni");
    ros::NodeHandle n;
    // set initial parameters
    //    ParaAll    rmin rmax,(g),(b), distmin,distmax
    ros::param::param<std::string>("/MyParam",ParamAll,"90 255 0 50 0 50 0.3 1.5 100 0.01");
   // ros::param::set("/MyParam","100 255 0 70 0 70 0.3 1.2 100 0.01"); //rgb
    ros::param::set("/MyParam","0 18 80 255 10 250 0 1.2 100 0.01"); //hsi
    ros::param::set("/Clusternum","1");
    ros::param::set("/gvector","0.0 0.0 2.0 0.0 1.0 2.45");
    PCLviewer = rgbVis();
    //1000 equals to the message queue
    //dont be too large or it will become slow
    ros::Subscriber sub = n.subscribe("/camera_remote/depth_registered/points",1,CloudCallBack);
    ros::Subscriber sub1  = n.subscribe("/camera1_remote/depth_registered/points",1,HandEye);

    //bagfile
   // ros::Subscriber sub = n.subscribe("/camera_remote_uncalibrated/depth_registered/points",1,CloudCallBack);
   // ros::Subscriber sub1  = n.subscribe("/camera1/depth_registered/points",1,HandEye);

    tf::TransformListener listener;
    tf::Vector3  gravity_vec;
    gravity_vec.setX(tfScalar(0));
    gravity_vec.setY(tfScalar(0));
    gravity_vec.setZ(tfScalar(-1));
    while (!PCLviewer->wasStopped ())
    {
        tf::StampedTransform transform,transformhandeye;
        ros::spinOnce();
        try{
            //listener.lookupTransform(
            //            "/camera_rgb_optical_frame",
            //            "/cxy/camera1_rgb_optical_frame",
             //           ros::Time(0), transformhandeye);

            listener.lookupTransform(
                        "/BODY",
                        "/camera_depth_optical_frame",
                        ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ros::Duration(0.1).sleep();
            continue;
        }

        HandeyetoCamera = transformhandeye * tf::Transform::getIdentity();
        tf::Vector3 gravity_in_camera = transform.getBasis() * gravity_vec;
        ostringstream ss;
        float z = gravity_in_camera.getZ();
        float y = - gravity_in_camera.getX();
        float x = 0;
        ss<< 0 <<" "<< 0 <<" "<< 0 <<" "<<x<<" "<<y<<" "<<z;
        std::string paramset(ss.str());
        ros::param::set("/gvector",paramset);

        PCLviewer->spinOnce (50);
        boost::this_thread::sleep (boost::posix_time::microseconds (50000));

    }
    return 0;
}

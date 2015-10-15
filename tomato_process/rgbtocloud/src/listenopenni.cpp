//jsk...........chen
/****************************
 * For debug u need to run -DCMAKE_BUILD_TYPE=Debug   in CMake
 * *****************************/
#include <listenopenni.h>


RGBToCloudViewer::RGBToCloudViewer(ros::NodeHandle& n, ros::NodeHandle& np) :
    n_(n),
    np_(np),
    PCLviewer(new pcl::visualization::PCLVisualizer ("RGBToCloudViewer")),
    detect(new TOMATODETECT()),
    viewchange(false),
    bn(1),
    picking_flag(false),
    box_switch(true),
    initflag(0),
    Tracktime(0),
    framecount(2),
    HandeyetoCamera(tf::Transform::getIdentity()),
    frame_divider(0)
{
  // root of camera, to estimate gravity vector
  np_.param<std::string>("camera_root_frame", camera_root_frame, "/BODY");

  gravity_vec.setX(tfScalar(0));
  gravity_vec.setY(tfScalar(0));
  gravity_vec.setZ(tfScalar(-1));

  // set initial parameters
  //    ParaAll    rmin rmax,(g),(b), distmin,distmax
  ros::param::param<std::string>(
      "/MyParam",
      ParamAll,
      "90 255 0 50 0 50 0.3 1.5 100 0.01");
  // ros::param::set("/MyParam","100 255 0 70 0 70 0.3 1.2 100 0.01"); //rgb
  ros::param::set("/MyParam","0 18 80 255 10 250 0 1.2 100 0.01"); //hsi
  ros::param::set("/Clusternum","1");
  ros::param::set("/gvector","0.0 0.0 2.0 0.0 1.0 2.45");

  pub_bb =
      n_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxsize", 1);
  pub_centroid =
      n_.advertise<geometry_msgs::PointStamped>("/centroid", 1);

  //1000 equals to the message queue
  //dont be too large or it will become slow
  sub =
      n_.subscribe("/camera_remote/depth_registered/points", 1,
                   &RGBToCloudViewer::CloudCallBack, this);
  sub1 =
      n_.subscribe("/camera1_remote/depth_registered/points", 1,
                   &RGBToCloudViewer::HandEye, this);

  //bagfile
  // sub = n_.subscribe("/camera_remote_uncalibrated/depth_registered/points",1,CloudCallBack);
  // sub1  = n_.subscribe("/camera1/depth_registered/points",1,HandEye);

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  PCLviewer->setBackgroundColor (1, 1, 1);
  PCLviewer->addCoordinateSystem (1.0);
  PCLviewer->initCameraParameters ();
  PCLviewer->registerKeyboardCallback(&RGBToCloudViewer::keyboardEvent, *this);
}

RGBToCloudViewer::~RGBToCloudViewer() {
}

/**************************
    Keyboard event
**************************/
void RGBToCloudViewer::keyboardEvent(
    const pcl::visualization::KeyboardEvent &event, void *)
{

  if(event.keyUp())
  {
    switch(event.getKeyCode())
    {
      case 'z':{
        //change bounding box....
        ostringstream ss;
        bn = (bn<detect->Center_vector.size()?bn+1:1);
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

        Aim_branch = detect->Center_vector.at(bn-1);
        picking_flag = true;
        tf_transmit(Aim_branch[0], Aim_branch[1], Aim_branch[2]);
        //then publish something to move the robot...

        break;
      }
    }
  }
}


void RGBToCloudViewer::easytracking()
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


void RGBToCloudViewer::viewerOneOff(
    pcl::visualization::PCLVisualizer& viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

  easytracking();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
      rgb(cloud);
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

  // publish centroid
  ostringstream ss;
  float z = Center[2];
  float y = Center[1];
  float x = Center[0];
  ss<<x<<" "<<y<<" "<<z;
  std::string paramset(ss.str());
  ros::param::set("/Centroid",paramset);

  geometry_msgs::PointStamped centroid_msg;
  centroid_msg.header.frame_id = "/camera_rgb_optical_frame";
  centroid_msg.header.stamp = ros::Time::now();
  centroid_msg.point.x = Center[0];
  centroid_msg.point.y = Center[1];
  centroid_msg.point.z = Center[2];
  pub_centroid.publish(centroid_msg);

  for(int i=0;i<detect->Center_vector.size();i++){
    std::stringstream ss;
    ss << "branch" << i;
    pcl::ModelCoefficients cube_coeff;
    if(detect->Boundingbox.size()>i){
      cube_coeff = detect->Boundingbox[i];
      viewer.addCube(
          cube_coeff.values[0],  // x_min
          cube_coeff.values[1],  // x_max
          cube_coeff.values[2],  // y_min
          cube_coeff.values[3],  // y_max
          cube_coeff.values[4],  // z_min
          cube_coeff.values[5],  // z_max
          cube_coeff.values[6],  // r
          cube_coeff.values[7],  // g
          cube_coeff.values[8],  // b
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
void RGBToCloudViewer::HandEye(const sensor_msgs::PointCloud2 msg)
{
  // then convert the pc2 format to classic pcl::pc format
  pcl::fromROSMsg(msg,Handeye_cloud);
  Handeye_cloud =
      *(detect->HandCloudFilter(Handeye_cloud.makeShared()))->makeShared();
  pcl_ros::transformPointCloud(
      *Handeye_cloud.makeShared(), Handeye_cloud, HandeyetoCamera);

  //leap_cloud.header.frame_id =
  //viewerOneOff(*PCLviewer,Handeye_cloud.makeShared());
}

//call back function
void RGBToCloudViewer::CloudCallBack(const sensor_msgs::PointCloud2 msg)
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
  cloud_frame = *(detect->CloudIN(cloud.makeShared(),ParamAll))->makeShared();
  if(cloud_frame.points.size()>2){
    int index = detect->GetCenter(cloud_frame.makeShared());
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
        for(int i = 0;i<detect->Center_vector.size();i++){
          float tmp =
              sqrt(DistToCenter(detect->Center_vector.at(i), Aim_branch));
          if(tmp<disttmp){
            disttmp = tmp;
            index = i;
          }
        }
      }
      Center = detect->Center_vector.at(index);

      // publish bounding box
      std::stringstream ss;
      ss <<
          detect->Boundingbox[index].values[0] << " " <<  // x_min
          detect->Boundingbox[index].values[1] << " " <<  // x_max
          detect->Boundingbox[index].values[2] << " " <<  // y_min
          detect->Boundingbox[index].values[3] << " " <<  // y_max
          detect->Boundingbox[index].values[4] << " " <<  // z_min
          detect->Boundingbox[index].values[5];  // z_max
      std::string paramset(ss.str());
      if(box_switch)
        ros::param::set("/Boxsize",paramset);

      jsk_recognition_msgs::BoundingBoxArray jsk_bb;
      jsk_bb.header.frame_id = msg.header.frame_id;
      jsk_bb.header.stamp = msg.header.stamp;
      jsk_bb.boxes.resize(1);

      jsk_bb.boxes[0].header.frame_id = msg.header.frame_id;
      jsk_bb.boxes[0].header.stamp = msg.header.stamp;

      jsk_bb.boxes[0].pose.position.x =
          (detect->Boundingbox[index].values[0] +
           detect->Boundingbox[index].values[1]) * 0.5;
      jsk_bb.boxes[0].pose.position.y =
          (detect->Boundingbox[index].values[2] +
           detect->Boundingbox[index].values[3]) * 0.5;
      jsk_bb.boxes[0].pose.position.z =
          (detect->Boundingbox[index].values[4] +
           detect->Boundingbox[index].values[5]) * 0.5;

      jsk_bb.boxes[0].pose.orientation.x = 0.0;
      jsk_bb.boxes[0].pose.orientation.y = 0.0;
      jsk_bb.boxes[0].pose.orientation.z = 0.0;
      jsk_bb.boxes[0].pose.orientation.w = 1.0;

      jsk_bb.boxes[0].dimensions.x =
          (detect->Boundingbox[index].values[1] -
           detect->Boundingbox[index].values[0]);
      jsk_bb.boxes[0].dimensions.y =
          (detect->Boundingbox[index].values[3] -
           detect->Boundingbox[index].values[2]);
      jsk_bb.boxes[0].dimensions.z =
          (detect->Boundingbox[index].values[5] -
           detect->Boundingbox[index].values[4]);

      pub_bb.publish(jsk_bb);

    }
    //now we got the center
    //CLOUDviewer.runOnVisualizationThreadOnce(viewerOneOff);
  }
  cloud_frame = *(detect->CloudIN(cloud.makeShared(),ParamAll))->makeShared();
  //CLOUDviewer.showCloud(cloud.makeShared());
  if(viewchange)
    viewerOneOff(*PCLviewer,cloud_frame.makeShared());
  else
    viewerOneOff(*PCLviewer,cloud.makeShared());

  // }
  //else
  //    cloud_frame += *(detect->CloudIN(cloud.makeShared(),ParamAll))->makeShared();
}

void RGBToCloudViewer::ProcOnce() {
  tf::StampedTransform transform,transformhandeye;
  try{
    //listener.lookupTransform(
    //            "/camera_rgb_optical_frame",
    //            "/cxy/camera1_rgb_optical_frame",
    //           ros::Time(0), transformhandeye);

    listener.lookupTransform(
        camera_root_frame,
        "/camera_depth_optical_frame",
        ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s, skip proc...",ex.what());
    ros::Duration(0.1).sleep();
    // continue;
    return;
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"listenopenni");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  RGBToCloudViewer viewer(n, np);

  while (ros::ok() && !viewer.wasStopped())
  {
    ros::spinOnce();
    viewer.ProcOnce();
    viewer.spinOnce(50);
    boost::this_thread::sleep (boost::posix_time::microseconds (50000));

  }
  return 0;
}

//jsk...........chen
/****************************
    Trying to detect the plane that segmentate tomatoes
*****************************/
#include <reconstruct.h>

/*********************************************
        Get the Callback and visualize
*********************************************/
static boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
static std::string ParamCentroid,Paramlast,ParamBox;
//THIS FUNCTION WILL BE CALLED AT EVERY FRAME.   ABOUT 50MS?
static pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
static pcl::PointCloud<pcl::PointXYZRGB> cloud_all;
RECONPROC *TEST = new RECONPROC;
//viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Object viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
static Eigen::Vector4f Center;
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.removeAllPointClouds();
    //add pc
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
}
//call back function
void CloudCallBack(const sensor_msgs::PointCloud2 msg)
{
  ros::param::get("/Centroid",ParamCentroid);
  ros::param::get("/Boxsize",ParamBox);
  printf ("Param= %s \n %s \n", ParamCentroid.c_str(),ParamBox.c_str());
  if(ParamCentroid.compare(Paramlast))
  {
      pcl::PCLPointCloud2 pcl_pc;
      //firstly convert the msg::pointcloud to pcl::pc
      pcl_conversions::toPCL(msg,pcl_pc);
       pcl::PointCloud<pcl::PointXYZRGB> cloud;
      // then convert the pc2 format to classic pcl::pc format
      pcl::fromPCLPointCloud2(pcl_pc,cloud);
    //deal with tomato
      cloud_frame = *(TEST->CloudIN(cloud.makeShared(),
                                    ParamCentroid,ParamBox))->makeShared();

      cloud_all = *(TEST->ICP_CON(cloud_all.makeShared(),
                                  cloud_frame.makeShared()))->makeShared();

      viewerOneOff(*PCLviewer,cloud_all.makeShared());
  }
  Paramlast = ParamCentroid;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"icp_recon");
  ros::NodeHandle n;
  // set initial parameters
  //  2 param     center and box
  ros::param::param<std::string>("/Centroid",ParamCentroid,
                                 "0.0 0.0 0.0");   //0,0,0
  ros::param::set("/Centroid","0.0 0.0 0.0");

  ros::param::param<std::string>("/Boxsize",ParamBox,
                                 "0.5 0.2 0.5");   //0.5,0.8,0.5
  ros::param::set("/Boxsize","0.5 0.2 0.5");

  PCLviewer = rgbVis();
  //1000 equals to the message queue
  //dont be too large or it will become slow
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points",1,CloudCallBack);

  while (!PCLviewer->wasStopped ())
  {
    ros::spinOnce();
    PCLviewer->spinOnce (10);
    boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }

  pcl::io::savePCDFileASCII ("PointcloudafterICP.pcd",  cloud_all);
  return 0;
}

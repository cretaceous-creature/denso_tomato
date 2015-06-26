//jsk...........chen
/****************************
    Trying to detect the plane that segmentate tomatoes
*****************************/
#include <boundary.h>

/*********************************************
        Get the Callback and visualize
*********************************************/
static boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
static std::string ParamBOUNDARY,Paramlast;
//THIS FUNCTION WILL BE CALLED AT EVERY FRAME.   ABOUT 50MS?
static pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
BOUNDARYPROC *TEST = new BOUNDARYPROC;
//viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
static Eigen::Vector4f Center;

//call back function
void CloudCallBack(const sensor_msgs::PointCloud2 msg)
{
  ros::param::get("/BOUNDARYParam",ParamBOUNDARY);
  printf ("Param= %s \n", ParamBOUNDARY.c_str());
  if(ParamBOUNDARY.compare(Paramlast))
  {
      pcl::PCLPointCloud2 pcl_pc;
      //firstly convert the msg::pointcloud to pcl::pc
      pcl_conversions::toPCL(msg,pcl_pc);
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      // then convert the pc2 format to classic pcl::pc format
      pcl::fromPCLPointCloud2(pcl_pc,cloud);

    //deal with tomato
      TEST->CloudIN(cloud.makeShared(),ParamBOUNDARY);

  }
  Paramlast = ParamBOUNDARY;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"listenopenni");
  ros::NodeHandle n;
  // set initial parameters
  //    ParaAll    rmin rmax,(g),(b), distmin,distmax
  ros::param::param<std::string>("/BOUNDARYParam",ParamBOUNDARY,
                                 "0 255 0 255 0 255 0.0 0.0 0.0 0.0 0.0 0.0");
  ros::param::set("/BOUNDARYParam","0 255 0 255 0 255 0.0 0.0 0.0 0.0 0.0 0.0");
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
  return 0;
}

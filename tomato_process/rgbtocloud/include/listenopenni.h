//jsk...............chen
// for tomato challenge
#ifndef LISTENOPENNI_H
#define LISTENOPENNI_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcproc.h>
#include <pcl/common/common_headers.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "geometry_msgs/PointStamped.h"

#define TrackDist 0.3


class RGBToCloudViewer {
 public:
  RGBToCloudViewer(ros::NodeHandle& n, ros::NodeHandle& np);
  ~RGBToCloudViewer();

  void HandEye(const sensor_msgs::PointCloud2 msg);
  void CloudCallBack(const sensor_msgs::PointCloud2 msg);

  void keyboardEvent(const pcl::visualization::KeyboardEvent& event, void *);
  void easytracking();
  void viewerOneOff(pcl::visualization::PCLVisualizer& viewer,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  void ProcOnce();

  inline void spinOnce(unsigned int sleep_time) {
    PCLviewer->spinOnce (sleep_time);
  }
  inline bool wasStopped() {
    return PCLviewer->wasStopped();
  }

  inline void tf_transmit(float x, float y, float z) {
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(x, y, z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(),
                             "camera_rgb_optical_frame" , "tomato"));
  }

  inline float DistToCenter(Eigen::Vector4f centroid,
                            Eigen::Vector4f centroid_last) {
    return (((centroid[0] - centroid_last[0]) *
             (centroid[0] - centroid_last[0])) +
            ((centroid[1] - centroid_last[1]) *
             (centroid[1] - centroid_last[1])) +
            ((centroid[2] - centroid_last[2]) *
             (centroid[2] - centroid_last[2])));
  }


 private:
  ros::NodeHandle n_;
  ros::NodeHandle np_;

  /*********************************************
        Get the Callback and visualize
  *********************************************/
  //static pcl::visualization::CloudViewer CLOUDviewer ("SIMPLE CLOUD VIEWER");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
  std::string ParamAll;
  TOMATODETECT *detect;
  //THIS FUNCTION WILL BE CALLED AT EVERY FRAME.   ABOUT 50MS?
  pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
  pcl::PointCloud<pcl::PointXYZRGB> Handeye_cloud;
  int counter;

  /**************************
    Keyboard event
  **************************/
  bool viewchange;
  int bn;
  Eigen::Vector4f Aim_branch;
  bool picking_flag;
  bool box_switch;

  Eigen::Vector4f Center,Center_last;
  int initflag;
  int Tracktime;

  int framecount;
  ros::Publisher pub_centroid;

  tf::Transform HandeyetoCamera;

  int frame_divider;
  ros::Publisher pub_bb;

  ros::Subscriber sub;
  ros::Subscriber sub1;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Vector3  gravity_vec;

  std::string camera_root_frame;
};

#endif

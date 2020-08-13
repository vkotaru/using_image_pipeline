#include <nodelet/nodelet.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/thread.hpp>
namespace using_image_pipeline {

class PclSegmentation {
public:
PclSegmentation*
//   ros::NodeHandlePtr nh_ptr_;
//   boost::shared_ptr<sensor_msgs::PointCloud2> cloud_;
//   boost::mutex connect_mutex_;

//   bool receivedAtLeastOnce = false;
//   ros::Subscriber sub_points_;
//   ros::Publisher pub_planes_;
//   ros::Publisher pub_point_height_;
//   std_msgs::Float32MultiArray msg_planes_;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, cloud, cloud_f;

//   PclSegmentationNodelet(/* args */) {
//     NODELET_INFO_STREAM("PclSegmentationNodelet constructor");
//   }
//   ~PclSegmentationNodelet() {}
//   virtual void onInit();
//   void pointCb(const sensor_msgs::PointCloud2ConstPtr &points_msg);
//   void connectCb();
//   void planarSegmentation();
// };

// void PclSegmentationNodelet::onInit() {
  
//   ros::NodeHandle &nh = getNodeHandle();
//   // nh_ptr_.reset(new ros::NodeHandle(nh));

//   NODELET_INFO_STREAM("Initialising nodelet... [PclSegmentationNodelet]");
//   sub_points_ = nh.subscribe("points", 3, &PclSegmentationNodelet::pointCb, this);

//   pub_planes_ = nh.advertise<std_msgs::Float32MultiArray>("planes", 3);
//   pub_point_height_ = nh.advertise<std_msgs::Float32>("test_msg", 3);

//   cloud_raw.reset(new pcl::PointCloud<pcl::PointXYZ>);
//   cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
// }

// // void PclSegmentationNodelet::connectCb() {
// //   boost::lock_guard<boost::mutex> lock(connect_mutex_);
// //   sub_points_ = nh_ptr_->subscribe("points", 3,
// //   &PclSegmentationNodelet::pointCb, this);
// // }

// void PclSegmentationNodelet::pointCb(
//     const sensor_msgs::PointCloud2ConstPtr &points_msg) {
//   // boost::mutex::scoped_lock lock (mutex_);

//   NODELET_DEBUG("PointCloud2 height %f", (double)points_msg->height);
//   std::cout << points_msg->height << std::endl;

//   // convert points_msg to pcl point cloud
//   pcl::fromROSMsg(*points_msg, *cloud_raw);
//     pcl::VoxelGrid<pcl::PointXYZ> vg;


//   std_msgs::Float32 tmp_msg;
//   tmp_msg.data = (float)cloud_raw->height;
//   pub_point_height_.publish(tmp_msg);

//   pub_planes_.publish(msg_planes_);
}

} // namespace using_image_pipeline

int main() {

    return 0;
}



#include <depth_image_proc/depth_conversions.h>
#include <image_transport/image_transport.h>
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
// #include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/thread.hpp>
namespace using_image_pipeline {

class PclSegmentationNodelet : public nodelet::Nodelet {
public:
  ros::NodeHandlePtr nh_ptr_;
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_;
  boost::mutex connect_mutex_;

  bool receivedAtLeastOnce = false;
  ros::Subscriber sub_points_;
  ros::Publisher pub_planes_;
  ros::Publisher pub_point_height_;
  std_msgs::Float32MultiArray msg_planes_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, cloud, cloud_f;

  PclSegmentationNodelet(/* args */) {
    NODELET_INFO_STREAM("PclSegmentationNodelet constructor");
  }
  ~PclSegmentationNodelet() {}
  virtual void onInit();
  void pointCb(const sensor_msgs::PointCloud2ConstPtr &points_msg);
  void connectCb();
  void planarSegmentation();
};

void PclSegmentationNodelet::onInit() {
  ros::NodeHandle &nh = getNodeHandle();
  nh_ptr_.reset(new ros::NodeHandle(nh));
  // cloud_.reset(new sensor_msgs::PointCloud2(nh));

  NODELET_INFO_STREAM("Initialising nodelet... [PclSegmentationNodelet]");
  ros::SubscriberStatusCallback connect_cb = boost::bind(&PclSegmentationNodelet::connectCb, this);
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_planes_ = nh_ptr_->advertise<std_msgs::Float32MultiArray>("planes", 3, connect_cb, connect_cb);
  pub_point_height_ = nh_ptr_->advertise<std_msgs::Float32>("test_msg", 3, connect_cb, connect_cb);

  cloud_raw.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

}

void PclSegmentationNodelet::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  sub_points_ = nh_ptr_->subscribe("points", 3, &PclSegmentationNodelet::pointCb, this);
}

void PclSegmentationNodelet::pointCb(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
  NODELET_DEBUG("PointCloud2 height %f", (double)points_msg->height);
  std::cout << points_msg->height << std::endl;

  // convert points_msg to pcl point cloud
  pcl::fromROSMsg(*points_msg, *cloud_raw);

  std_msgs::Float32 tmp_msg;
  tmp_msg.data = (float)cloud_raw->height;
  pub_point_height_.publish(tmp_msg);

  pub_planes_.publish(msg_planes_);
}

void PclSegmentationNodelet::planarSegmentation() {

  // segment planes
//   planarSegmentation();
  // remove NaN from the point-cloud
//   std::vector<int> tmp;
//   pcl::removeNaNFromPointCloud(*cloud_raw, *cloud, tmp);

//   Create the filtering object: downsample the dataset using a leaf size of 1cm
//   float leaf_size = 0.01; // 10cm
//   pcl::VoxelGrid<pcl::PointXYZ> vg;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//   vg.setInputCloud(cloud_raw);
//   vg.setLeafSize(leaf_size, leaf_size, leaf_size);
//   vg.filter(*cloud_filtered);

//   // Create the segmentation object for the planar model and set all the
//   // parameters
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//   pcl::PCDWriter writer;
//   seg.setOptimizeCoefficients(true);
//   seg.setModelType(pcl::SACMODEL_PLANE);
//   seg.setMethodType(pcl::SAC_RANSAC);
//   // seg.setMaxIterations(1000);
//   seg.setDistanceThreshold(0.01);

//   int i = 0, nr_points = (int)cloud_filtered->size();
//   while (cloud_filtered->size() > 0.3 * nr_points) {
//     // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud(cloud_filtered);
//     seg.segment(*inliers, *coefficients);
//     if (inliers->indices.size() == 0) {
//       std::cout << "Could not estimate a planar model for the given dataset."
//                 << std::endl;
//       break;
//     }
//     std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//               << coefficients->values[1] << " " << coefficients->values[2]
//               << " " << coefficients->values[3] << std::endl;
//     for (int k = 0; k < 4; k++) {
//       msg_planes_.data[i * 4 + k] = coefficients->values[k];
//     }

//     // Extract the planar inliers from the input cloud
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud(cloud_filtered);
//     extract.setIndices(inliers);
//     extract.setNegative(false);

//     // Remove the planar inliers, extract the rest
//     extract.setNegative(true);
//     extract.filter(*cloud_f);
//     *cloud_filtered = *cloud_f;
//     i++;
//   }
}

} // namespace using_image_pipeline

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(using_image_pipeline::PclSegmentationNodelet, nodelet::Nodelet);
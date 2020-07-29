#include <depth_image_proc/depth_conversions.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>

#include <boost/thread.hpp>
namespace using_image_pipeline {

class PclSegmentationNodelet : public nodelet::Nodelet {
   public:
    ros::NodeHandlePtr nh_ptr_;
    boost::shared_ptr<sensor_msgs::PointCloud2> cloud_;
    boost::mutex connect_mutex_;
    bool receivedAtLeastOnce = false;
    ros::Subscriber sub_points_;
    ros::Publisher pub_point_height_;
    int i = 0;

    PclSegmentationNodelet(/* args */) {
        NODELET_INFO_STREAM("PclSegmentationNodelet constructor");
    }
    ~PclSegmentationNodelet() {}
    virtual void onInit();
    void pointCb(const sensor_msgs::PointCloud2ConstPtr& points_msg);
    void connectCb();
};

void PclSegmentationNodelet::onInit() {
    ros::NodeHandle& nh = getNodeHandle();
    nh_ptr_.reset(new ros::NodeHandle(nh));
    // cloud_.reset(new sensor_msgs::PointCloud2(nh));

    NODELET_INFO_STREAM("Initialising nodelet... [PclSegmentationNodelet]");
    ros::SubscriberStatusCallback connect_cb = boost::bind(&PclSegmentationNodelet::connectCb, this);
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_point_height_ = nh_ptr_->advertise<std_msgs::Float32>("test_msg", 3, connect_cb, connect_cb);
}

void PclSegmentationNodelet::pointCb(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_DEBUG("PointCloud2 height %f", (double)points_msg->height);
    std::cout << points_msg->height << std::endl;
    std_msgs::Float32 tmp_msg;
    tmp_msg.data = (float)points_msg->height;
    pub_point_height_.publish(tmp_msg);

    if (!receivedAtLeastOnce) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud (new pcl::PointCloud<pcl::PointXYZ>); // TODO: (@vkotaru: find out why this instantiation is important)
        pcl::visualization::PCLVisualizer viewer("Sample Cloud Viewer");

        pcl::fromROSMsg(*points_msg, *points_cloud);
        viewer.addPointCloud(points_cloud, "pcl_points");
        pcl::io::savePCDFile("/home/kotaru/catkin_ws/cloud_" + std ::to_string(i)+".pcd", *points_cloud, true);
        i++;
        // viewer.spin();  FIXME: run pcl visualization in a separate thread
        receivedAtLeastOnce = true;
    }
}

void PclSegmentationNodelet::connectCb() {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    sub_points_ = nh_ptr_->subscribe("points", 3, &PclSegmentationNodelet::pointCb, this);
}

}  // namespace using_image_pipeline

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(using_image_pipeline::PclSegmentationNodelet, nodelet::Nodelet);
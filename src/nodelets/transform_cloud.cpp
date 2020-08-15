#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pcl/common/transforms.h>
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

#include "eigen3/Eigen/Dense"

namespace using_image_pipeline {

class TransformCloudNodelet : public nodelet::Nodelet {
   public:
    ros::NodeHandlePtr nh_ptr_;
    boost::shared_ptr<sensor_msgs::PointCloud2> cloud_;
    boost::mutex connect_mutex_;
    boost::mutex mutex_;

    typedef sensor_msgs::PointCloud2 PointCloud;

    ros::Subscriber sub_points_;
    ros::Publisher pub_transformed_points_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, cloud_transformed;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    ddynamic_reconfigure::DDynamicReconfigure ddynrec_;

    TransformCloudNodelet(/* args */) { NODELET_INFO_STREAM("TransformCloudNodelet constructor"); }
    ~TransformCloudNodelet() {}
    virtual void onInit();
    void pointCb(const sensor_msgs::PointCloud2ConstPtr &points_msg);
    void connectCb();
};

void TransformCloudNodelet::onInit() {
    ros::NodeHandle &nh = getNodeHandle();
    nh_ptr_.reset(new ros::NodeHandle(nh));

    NODELET_INFO_STREAM("Initialising nodelet... [TransformCloudNodelet]");
    ros::SubscriberStatusCallback connect_cb = boost::bind(&TransformCloudNodelet::connectCb, this);
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_transformed_points_ = nh_ptr_->advertise<PointCloud>("points_transformed", 3, connect_cb, connect_cb);

    transform = Eigen::Matrix4f::Zero();
    transform(0, 2) = 1;
    transform(1, 0) = 1;
    transform(2, 1) = 1;
    transform(3, 3) = 1;
    std::cout << "Transform: \n" << transform << std::endl;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ddynrec_.registerVariable<double>(
                "channel_" + std::to_string(i)+ std::to_string(j), transform(i, j),
                [this, i, j](int new_value) { this->transform(i, j) = new_value; }, "transform matrix", -1, 1);
        }
    }
    ddynrec_.publishServicesTopics();


    cloud_raw.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void TransformCloudNodelet::connectCb() {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    sub_points_ = nh_ptr_->subscribe("points", 3, &TransformCloudNodelet::pointCb, this);
}

void TransformCloudNodelet::pointCb(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
    PointCloud::Ptr cloud_transformed_msg(new PointCloud);

    // convert points_msg to pcl point cloud
    pcl::fromROSMsg(*points_msg, *cloud_raw);
    pcl::transformPointCloud(*cloud_raw, *cloud_transformed, transform);
    pcl::toROSMsg(*cloud_transformed, *cloud_transformed_msg);

    pub_transformed_points_.publish(cloud_transformed_msg);
}

}  // namespace using_image_pipeline

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(using_image_pipeline::TransformCloudNodelet, nodelet::Nodelet);
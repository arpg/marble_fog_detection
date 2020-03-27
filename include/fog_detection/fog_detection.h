#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h> // pub, sub images in ROS
#include <cv_bridge/cv_bridge.h> // useful functions for image encodings
#include <sensor_msgs/image_encodings.h> // for ROS-OpenCV conversion: toCvShare, toCvCopy
#include <opencv2/imgproc/imgproc.hpp> // image processing
#include <opencv2/highgui/highgui.hpp> // GUI modules

#include <iostream>

#include <dynamic_reconfigure/server.h>

#include <fog_detection/FogDetectionConfig.h>

#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <limits.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

#include <geometry_msgs/TwistStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <visualization_msgs/Marker.h>

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/features/obstacles.h"
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/extract_indices.h>

#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef PointCloud::Ptr ptr_cloud;

using namespace cv;
using namespace cv_bridge; // CvImage, toCvShare


namespace fog
{
  class FogDetectionNodelet : public nodelet::Nodelet
  {
    public:
      // ROS communication
      boost::shared_ptr<image_transport::ImageTransport> it_in_;
      ros::Subscriber sub_low_depth_image_;
      ros::Subscriber sub_low_depth_pcl_;

      ros::Publisher pub_output_;
      ros::Publisher pub_low_pcl_;
      ros::Publisher pub_low_image_out_;

      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

      boost::mutex connect_mutex_;

      int queue_size_;
      std::string target_frame_id_;

      // Dynamic reconfigure
      boost::recursive_mutex config_mutex_;
      typedef fog_detection::FogDetectionConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      boost::shared_ptr<ReconfigureServer> reconfigure_server_;
      Config config_;

      void onInit();

      // Handles (un)subscribing when clients (un)subscribe;
      void connectCb();

      void configCb(Config &config, uint32_t level);

      void low_depth_image_cb(const sensor_msgs::ImageConstPtr& image_msg);

      void low_point_cloud_cb(const PointCloud::ConstPtr& msg);

      void depth_image_to_twist(const sensor_msgs::ImageConstPtr& image_msg,
                                const ros::Publisher pub_pcl_,
                                int x_offset,
                                int y_offset,
                                int width,
                                int height);

      void point_cloud_to_twist(const PointCloud::ConstPtr& cloud_in,
		      		const ros::Publisher pub_pcl_);

    private:

      // Depth Camera Frames
      std::string low_robot_frame;
      std::string low_sensor_frame;

      // Depth Image Parameters
      int low_camera_pixel_x_offset;
      int low_camera_pixel_y_offset;
      int low_camera_pixel_width;
      int low_camera_pixel_height;

      // Point Cloud Parameters
      bool low_publish_pointcloud;

      tf::TransformListener low_listener;
      tf::StampedTransform low_transform;
      tf::Matrix3x3 low_m_euler;
      tf::Matrix3x3 low_m_inv_euler;
      double low_roll;
      double low_pitch;
      double low_yaw;

      // Point Cloud Sequence
      long int seq = 0;

      float transform_pcl_roll_;
      float transform_pcl_pitch_;
      float transform_pcl_yaw_;
      float normal_radius_;
      float normal_x_LT_threshold_;
      float normal_x_GT_threshold_;
      float normal_y_LT_threshold_;
      float normal_y_GT_threshold_;
      float normal_z_LT_threshold_;
      float normal_z_GT_threshold_;
      float intensity_LT_threshold_;
      float intensity_GT_threshold_;
      float sor_nearest_neighbors_;
      float sor_std_dev_multiplier_;
      float ror_radius_;
      float ror_min_neighbors_;
      float height_variance_radius_;

      pcl::search::Search<pcl::PointXYZ>::Ptr tree_xyz;
      typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudOut;

  };

};

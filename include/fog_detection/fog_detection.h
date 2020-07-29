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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <ouster/os1.h>
#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/os1_ros.h>

#include <chrono> 
using namespace std::chrono; 
using namespace cv;
using namespace cv_bridge; // CvImage, toCvShare
namespace OS1 = ouster::OS1;

namespace fog
{
  class FogDetectionNodelet : public nodelet::Nodelet
  {
    public:
    
      // ROS communication
      ros::Subscriber sub_pcl_;
      ros::Publisher pub_foreground_pcl_;
      ros::Publisher pub_foreground_z_pcl_;
      ros::Publisher pub_foreground_z_ror_pcl_;
      ros::Publisher pub_fog_pcl_;

      ros::Publisher pub_range_img_;
      ros::Publisher pub_intensity_img_;

      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

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

      void range_image_cb(const sensor_msgs::ImageConstPtr& image_msg);
      void intensity_image_cb(const sensor_msgs::ImageConstPtr& image_msg);
      void point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

      double binarySearch(std::vector<double>& array, const double value, const double threshold);

      void linearSpacedArray(std::vector<double> &xs, double a, double b, std::size_t N);

      void getDepthImageCPFL(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2, cv::Mat &range_img, cv::Mat &index_img);

      void getDepthImageOfficial(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_ros,
                                cv::Mat &range_img);

      void getPreFilterImage(cv::Mat &range_img, cv::Mat &filter_img);

      void labelForegroundFilterPCL(pcl::PointXYZI search_pt,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2,
                                    pcl::PointIndices::Ptr inliers, 
                                    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree,
                                    int i);

      void getPostFilterPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2,
                            pcl::PointIndices::Ptr inliers);

    private:
 
      float fog_min_range_deviation_;
      float fog_radius_high_intensity_;
      float fog_low_intensity_;
      float fog_high_intensity_;

      cv::Mat last_range_img;

            // If true, Official Ouster ROS driver, publishes pcl with x, y, z, range, noise, ring, reflectivity, and t fields.
      // If false, CPFL-forked ROS driver, publishes pcl with x, y, z, intensity fields;
      bool flag_official_ouster_pcl; 

      // Width and height of the point cloud
      int W;
      int H;

      // Look up tables for polar coordinate angle lookups
      std::vector<double> azim_LUT;
      std::vector<double> elev_LUT;

      // Pixel offset (only for official Ouster ROS driver)
      std::vector<int> px_offset;

  };

};

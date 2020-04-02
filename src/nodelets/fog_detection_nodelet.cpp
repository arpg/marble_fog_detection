#include <fog_detection/fog_detection.h>

namespace fog
{
    void FogDetectionNodelet::connectCb()
    {
    };

    void FogDetectionNodelet::onInit()
    {
        nh         = getMTNodeHandle();
        private_nh = getMTPrivateNodeHandle();

        it_in_ .reset(new image_transport::ImageTransport(nh));

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("target_frame_id", target_frame_id_, std::string());

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&FogDetectionNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

	    // Monitor whether anyone is subscribed to the h_scans
        image_transport::SubscriberStatusCallback connect_cb = boost::bind(&FogDetectionNodelet::connectCb, this);
        ros::SubscriberStatusCallback connect_cb_info = boost::bind(&FogDetectionNodelet::connectCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to pub_h_scans_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());

        // Subscriber Ouster images
        sub_range_img_ = nh.subscribe("in_range_img", 1, &FogDetectionNodelet::range_image_cb, this);
        sub_intensity_img_ = nh.subscribe("in_intensity_img", 1, &FogDetectionNodelet::intensity_image_cb, this);

        sub_low_depth_pcl_ = nh.subscribe("down/point_cloud", 1, &FogDetectionNodelet::low_point_cloud_cb, this);

        // Publish Point Cloud
        pub_low_pcl_            = private_nh.advertise<PointCloud>("output", 10);
        pub_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_range_img", 10);
        pub_intensity_img_      = private_nh.advertise<sensor_msgs::Image>("out_intensity_img", 10);

        // Create static tf broadcaster (-30 pitch, Realsense pointed down)
        // rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 -0.00913852259 0.0 base_link royale_camera_optical_frame 1000

        try
        {
            low_listener.waitForTransform(low_sensor_frame, low_robot_frame, ros::Time(0), ros::Duration(10.0) );
            low_listener.lookupTransform(low_sensor_frame, low_robot_frame, ros::Time(0), low_transform);
            low_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	        low_m_euler.setRotation(low_transform.getRotation());
            int low_solution_number = 1;
            low_m_euler.getEulerYPR(low_yaw, low_pitch, low_roll, low_solution_number);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
        }

        std::cout << "base_link->camera roll: " << low_roll << std::endl;
        std::cout << "base_link->camera pitch: " << low_pitch << std::endl;
        std::cout << "base_link->camera yaw: " << low_yaw << std::endl;

    };

    void FogDetectionNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;

        // Depth Camera Parameters
        low_robot_frame            = config.low_robot_frame;
        low_sensor_frame           = config.low_sensor_frame;

        // Depth Image Parameters
        low_camera_pixel_x_offset  = config.low_camera_pixel_x_offset;
        low_camera_pixel_y_offset  = config.low_camera_pixel_y_offset;
        low_camera_pixel_width     = config.low_camera_pixel_width;
        low_camera_pixel_height    = config.low_camera_pixel_height;

        transform_pcl_roll_         = config.transform_pcl_roll;
        transform_pcl_pitch_        = config.transform_pcl_pitch;
        transform_pcl_yaw_          = config.transform_pcl_yaw;
        normal_radius_              = config.normal_radius;
        normal_x_LT_threshold_      = config.normal_x_LT_threshold;
        normal_x_GT_threshold_      = config.normal_x_GT_threshold;
        normal_y_LT_threshold_      = config.normal_y_LT_threshold;
        normal_y_GT_threshold_      = config.normal_y_GT_threshold;
        normal_z_LT_threshold_      = config.normal_z_LT_threshold;
        normal_z_GT_threshold_      = config.normal_z_GT_threshold;
        ror_radius_                 = config.ror_radius;
        ror_min_neighbors_          = config.ror_min_neighbors;

    };

    void FogDetectionNodelet::range_image_cb(const sensor_msgs::ImageConstPtr& image_msg)
    {
        analyze_range_images(image_msg,
                             pub_range_img_,
                             low_camera_pixel_x_offset,
                             low_camera_pixel_y_offset,
                             low_camera_pixel_width,
                             low_camera_pixel_height
                             );
    };

    void FogDetectionNodelet::intensity_image_cb(const sensor_msgs::ImageConstPtr& image_msg)
    {
        analyze_intensity_images(image_msg,
                             pub_intensity_img_,
                             low_camera_pixel_x_offset,
                             low_camera_pixel_y_offset,
                             low_camera_pixel_width,
                             low_camera_pixel_height
                             );
    };

    void FogDetectionNodelet::low_point_cloud_cb(const PointCloud::ConstPtr& cloud_in)
    {
        point_cloud_to_twist(cloud_in,
                             pub_low_pcl_
                             );
    };

    void FogDetectionNodelet::analyze_range_images(const sensor_msgs::ImageConstPtr& in_msg,
                                                   const ros::Publisher pub_img_,
                                                   int x_offset,
                                                   int y_offset,
                                                   int width,
                                                   int height)
    {

        // Get cropping parameters
        int max_width = in_msg->width - x_offset;
        int max_height = in_msg->height - y_offset;
        if (width == 0 || width > max_width)
            width = max_width;
        if (height == 0 || height > max_height)
            height = max_height;

        // Convert from ROS to OpenCV
        CvImageConstPtr source = toCvShare(in_msg);

        // Crop image
        new_range_img = source->image(cv::Rect(x_offset, y_offset, width, height));

        // https://github.com/IntelRealSense/librealsense/issues/3286
        if (in_msg->encoding != "32FC1")
        {

            // The intensity image is of type CV_8UC1 (0-255, 0 is no return, higher is closer to sensor)
            new_range_img.convertTo(new_range_img,CV_32FC1, 0.39215686274); // 100/255 = 0.39215686274
        }

        diff_range_img = new_range_img - old_range_img;

        // std::cout << "diff_range_img = " << std::endl << " "  << diff_range_img << std::endl << std::endl;

        cv_bridge::CvImage out_msg;
        out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_msg.image    = diff_range_img; // Your cv::Mat

        pub_img_.publish(out_msg.toImageMsg());

        old_range_img = new_range_img;

    };

    void FogDetectionNodelet::analyze_intensity_images(const sensor_msgs::ImageConstPtr& in_msg,
                                                       const ros::Publisher pub_img_,
                                                       int x_offset,
                                                       int y_offset,
                                                       int width,
                                                       int height)
    {

        // Get cropping parameters
        int max_width = in_msg->width - x_offset;
        int max_height = in_msg->height - y_offset;
        if (width == 0 || width > max_width)
            width = max_width;
        if (height == 0 || height > max_height)
            height = max_height;

        // Convert from ROS to OpenCV
        CvImageConstPtr source = toCvShare(in_msg);

        // Crop image
        new_intensity_img = source->image(cv::Rect(x_offset, y_offset, width, height));

        // The intensity image is of type CV_8UC1 (0-255)
        new_intensity_img.convertTo(new_intensity_img, CV_32FC1, 0.39215686274);

        // https://github.com/IntelRealSense/librealsense/issues/3286
        // if (in_msg->encoding != "32FC1")
        // {
        //     new_intensity_img.convertTo(new_intensity_img,CV_32FC1, 0.39215686274); // 100/255 = 0.39215686274
        // }

        diff_intensity_img = new_intensity_img - old_intensity_img;

        std::cout << "diff_intensity_img" << std::endl << " "  << diff_intensity_img << std::endl << std::endl;

        cv_bridge::CvImage out_msg;
        out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_msg.image    = diff_intensity_img; // Your cv::Mat

        pub_img_.publish(out_msg.toImageMsg());

        old_intensity_img = new_intensity_img;

    };

    void FogDetectionNodelet::point_cloud_to_twist(const PointCloud::ConstPtr& cloud_in,
                                                       const ros::Publisher pub_pcl_)
    {
        //////////////////////////////////////
        // FILTER OUT NANS FROM POINT CLOUD //
        //////////////////////////////////////

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<std::vector<int>> indices_nan(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud_in, *indices_nan);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(indices_nan);
        extract.setNegative(false);
        extract.filter(*cloud_in_filtered);

        /////////////////////////////////
        // TRANSFORM INPUT POINT CLOUD //
        /////////////////////////////////

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed (new pcl::PointCloud<pcl::PointXYZ>);
        tf::Transform transform_pcl;
        tf::Quaternion q;

        // Note roll and pitch are intentionally backwards due to the image frame to boldy frame transform_pcl. The IMU transform_pcl converts from body to world frame.
        // transform_pcl.setRotation( tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_) );
        transform_pcl.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
        pcl_ros::transformPointCloud(*cloud_in_filtered, *cloud_in_transformed, transform_pcl);

        ///////////////////////
        // CALCULATE NORMALS //
        ///////////////////////

        if (cloud_in_transformed->isOrganized ())
        {
            tree_xyz.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
        }
        else
        {
            // Use KDTree for non-organized data
            tree_xyz.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
        }

        // Set input pointcloud for search tree
        tree_xyz->setInputCloud(cloud_in_transformed);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> ne; // NormalEstimationOMP uses more CPU on NUC (do not use OMP!)
        ne.setInputCloud(cloud_in_transformed);
        ne.setSearchMethod(tree_xyz);

        // Set viewpoint, very important so normals are all pointed in the same direction
        ne.setViewPoint(std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ne (new pcl::PointCloud<pcl::PointXYZINormal>);

        ne.setRadiusSearch(normal_radius_);
        ne.compute(*cloud_ne);

        // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)
        for (int i = 0; i < cloud_in_transformed->points.size(); i++)
        {
            cloud_ne->points[i].x = cloud_in_transformed->points[i].x;
            cloud_ne->points[i].y = cloud_in_transformed->points[i].y;
            cloud_ne->points[i].z = cloud_in_transformed->points[i].z;
        }

        // Create conditional object
        pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointXYZINormal> () );

        // Add conditional statements
        // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
        // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );
        // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
        // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );

        // Build the filter
        pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud_ne);

        // Apply filter
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_condrem (new pcl::PointCloud<pcl::PointXYZINormal>);
        condrem.filter (*cloud_condrem);

        //////////////////
        // REDUCE NOISE //
        //////////////////

        // Build the filter
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ror (new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> ror;
        ror.setInputCloud(cloud_condrem);
        ror.setRadiusSearch(ror_radius_);
        ror.setMinNeighborsInRadius (ror_min_neighbors_);
        ror.filter (*cloud_ror);

        //////////////////////////////////
        // PUBLISH FILTERED POINT CLOUD //
        //////////////////////////////////

        seq++;
        cloud_ror->header.seq = seq;
        cloud_ror->header.frame_id = cloud_in->header.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), cloud_ror->header.stamp);
        pub_pcl_.publish (cloud_ror);

        // Clear memory
        cloud_ror->clear();

    };
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( fog::FogDetectionNodelet, nodelet::Nodelet)

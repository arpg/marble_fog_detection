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

        // sub_low_depth_pcl_ = nh.subscribe("in_pcl", 1, &FogDetectionNodelet::point_cloud_cb, this);
        sub_low_depth_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>("in_pcl", 500, boost::bind(&FogDetectionNodelet::point_cloud_cb, this, _1));

        // Publish Point Cloud
        pub_conf_pcl_               = private_nh.advertise<PointCloud>("out_conf_pcl", 10);
        pub_seg_pcl_                = private_nh.advertise<PointCloud>("out_seg_pcl", 10);
        pub_avg_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_avg_range_img", 10);
        pub_dev_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_dev_range_img", 10);
        pub_dev_diff_range_img_     = private_nh.advertise<sensor_msgs::Image>("out_dev_diff_range_img", 10);
        pub_var_t_img_              = private_nh.advertise<sensor_msgs::Image>("out_var_t_img", 10);
        pub_noreturn_img_           = private_nh.advertise<sensor_msgs::Image>("out_noreturn_img", 10);
        pub_noreturn_lowres_img_    = private_nh.advertise<sensor_msgs::Image>("out_noreturn_lowres_img", 10);
        pub_prob_noreturn_img_      = private_nh.advertise<sensor_msgs::Image>("prob_noreturn_img", 10);
        pub_range_img_              = private_nh.advertise<sensor_msgs::Image>("out_range_img", 10);
        pub_intensity_img_          = private_nh.advertise<sensor_msgs::Image>("out_intensity_img", 10);

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
        // analyze_range_images(image_msg,
        //                      pub_range_img_,
        //                      low_camera_pixel_x_offset,
        //                      low_camera_pixel_y_offset,
        //                      low_camera_pixel_width,
        //                      low_camera_pixel_height
        //                      );
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

    void FogDetectionNodelet::point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_ros)
    {
        ouster_ros::OS1::CloudOS1 cloud_in{};
        pcl::fromROSMsg(*cloud_in_ros, cloud_in);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_in_ros, *cloud_in2);

        // sensor_msgs::Image range_image;
        // sensor_msgs::Image noise_image;
        // sensor_msgs::Image intensity_image;

        // // Get PCL metadata
        // int W = cloud_in_ros->width;
        // int H = cloud_in_ros->height;
        // std::vector<int>  px_offset = ouster::OS1::get_px_offset(W);
        // // for 64 channels, should be [ 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 
        // //                              0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18]

        // // Setup Noise Image
        // noise_image.width = W;
        // noise_image.height = H;
        // noise_image.step = W;
        // noise_image.encoding = "mono8";
        // noise_image.data.resize(W * H);

        // // Set up Intensity Image
        // intensity_image.width = W;
        // intensity_image.height = H;
        // intensity_image.step = W;
        // intensity_image.encoding = "mono8";
        // intensity_image.data.resize(W * H);

        // cv::Mat zero_range_img(H, W, CV_32FC1, 0.1);
        // cv::Mat max_range_img(H, W, CV_32FC1, 20.0);
        // cv::Mat nonzero_range_img(H, W, CV_32FC1, 0.1);
        // cv::Mat range_img(H, W, CV_32FC1, 0.0);
        // cv::Mat avg_range_img(H, W, CV_32FC1, 0.0);
        // cv::Mat diff_range_img(H, W, CV_32FC1, 0.0);
        // cv::Mat var_s_raw_img(H, W, CV_32FC1, 0.0);
        // cv::Mat dev_range_img(H, W, CV_32FC1, 0.0);
        // cv::Mat dev_diff_range_img(H, W, CV_32FC1, 0.0);
        // cv::Mat var_t_img(H, W, CV_32FC1, 0.0);
        // cv::Mat return_img(H, W, CV_32FC1, 0.0);
        // cv::Mat return_sum_img(H, W, CV_32FC1, 0.0);
        // cv::Mat noreturn_img(H, W, CV_32FC1, 0.0);
        // cv::Mat prob_noreturn_img(H, W, CV_32FC1, 0.0);

        // cv::Mat zero_range_msk(H, W, CV_32FC1, 0.0);

        // cv::Mat noreturn_lowres_img;

        // for (int u = 0; u < H; u++) {
        //     for (int v = 0; v < W; v++) {
        //         const size_t vv = (v + px_offset[u]) % W;
        //         const size_t index = vv * H + u;
        //         const auto& pt = cloud_in[index];
        //         range_img.at<float>(u,v) = pt.range * 5e-3;
        //         noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);
        //         intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);
        //     }
        // }

        // if(last_range_img.rows == 0 && last_range_img.cols == 0)
        // {
        // }
        // else
        // {
        //     // Difference of range images
        //     diff_range_img = abs(range_img - last_range_img);
            
        //     // Binarize depth (range) image
        //     cv::threshold(range_img, return_img, 0.0, 1.0, THRESH_BINARY);

        //     // cv::blur(range_img, mean_s_img, cv::Size(9,9)); //Or whatever blurring you want

        //     // Average range image
        //     cv::boxFilter(range_img, avg_range_img, -1, cv::Size(9,9), cv::Point(-1,-1), false);

        //     // Average binarized range image (return image)
        //     cv::boxFilter(return_img, return_sum_img, -1, cv::Size(9,9), cv::Point(-1,-1), false);

        //     // Scale average range image
        //     cv::divide(avg_range_img, return_sum_img, avg_range_img);

        //     // Subtract range image from average range image, threshold betwneen 0 and 10
        //     cv::threshold(range_img - avg_range_img, dev_range_img, 0.0, 0.0, THRESH_TOZERO);
        //     cv::threshold(dev_range_img, dev_range_img, 10.0, 10.0, THRESH_TRUNC);

        //     double min = 0, max = 0;
        //     cv::Point minLoc(-1, -1), maxLoc(-1, -1);
        //     cv::minMaxLoc(dev_range_img, &min, &max, &minLoc, &maxLoc);
        //     std::cout << "min: " << min << std::endl;
        //     std::cout << "max: " << max << std::endl;

        //     // Compute difference between this frame and last frame (to remove dc content aka static gradients)
        //     dev_diff_range_img = abs(dev_range_img - last_dev_range_img);

        //     // Accumulate
        //     var_t_img = 0.9 * last_var_t_img;
        //     var_t_img = var_t_img + diff_range_img;

        //     // Compute binary no-return image (1 = no return, 0 = return)
        //     cv::threshold(range_img, noreturn_img, 0.0, 1.0, THRESH_BINARY_INV);

        //     // Average noreturn down to single row
        //     cv::Mat no_return_arr;
        //     cv::reduce(noreturn_img, no_return_arr, 0, CV_REDUCE_AVG, -1);

        //     // Resize down to less columns
        //     resize(noreturn_img, noreturn_lowres_img, cv::Size(48,4), 0, 0, INTER_AREA);
            
        //     // std::cout << "no_return_arr: " << no_return_arr << std::endl;

        //     // Accumulate
        //     prob_noreturn_img = 0.9 * last_prob_noreturn_img;
        //     prob_noreturn_img = prob_noreturn_img + noreturn_img;

        //     // https://stackoverflow.com/questions/18233691/how-to-index-and-modify-an-opencv-matrix
        // }

        // // Publish Range Image
        // cv_bridge::CvImage new_range_msg;
        // new_range_msg.encoding                  = sensor_msgs::image_encodings::TYPE_32FC1;
        // new_range_msg.image                     = range_img;
        // new_range_msg.header.stamp              = ros::Time::now();
        // new_range_msg.header.frame_id           = cloud_in_ros->header.frame_id;        
        // pub_range_img_.publish(new_range_msg.toImageMsg());

        // // Publish Range Avg Image
        // cv_bridge::CvImage avg_range_msg;
        // avg_range_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
        // avg_range_msg.image                      = avg_range_img;
        // avg_range_msg.header.stamp               = ros::Time::now();
        // avg_range_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
        // pub_avg_range_img_.publish(avg_range_msg.toImageMsg());

        // // Publish Range Deviation Image
        // cv_bridge::CvImage dev_range_msg;
        // dev_range_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
        // dev_range_msg.image                      = dev_range_img;
        // dev_range_msg.header.stamp               = ros::Time::now();
        // dev_range_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
        // pub_dev_range_img_.publish(dev_range_msg.toImageMsg());

        // // Publish Range Deviation Diff Image
        // cv_bridge::CvImage dev_diff_range_msg;
        // dev_diff_range_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
        // dev_diff_range_msg.image                      = dev_diff_range_img;
        // dev_diff_range_msg.header.stamp               = ros::Time::now();
        // dev_diff_range_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
        // pub_dev_diff_range_img_.publish(dev_diff_range_msg.toImageMsg());
        
        // // Publish Variance (in Time) Image
        // cv_bridge::CvImage var_t_msg;
        // var_t_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
        // var_t_msg.image                      = var_t_img;
        // var_t_msg.header.stamp               = ros::Time::now();
        // var_t_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
        // pub_var_t_img_.publish(var_t_msg.toImageMsg());

        // // Publish No Return Image
        // cv_bridge::CvImage noreturn_msg;
        // noreturn_msg.encoding               = sensor_msgs::image_encodings::TYPE_32FC1;
        // noreturn_msg.image                  = noreturn_img;
        // noreturn_msg.header.stamp           = ros::Time::now();
        // noreturn_msg.header.frame_id        = cloud_in_ros->header.frame_id;        
        // pub_noreturn_img_.publish(noreturn_msg.toImageMsg());

        // // Publish No Return (Low Res) Image
        // cv_bridge::CvImage noreturn_lowres_msg;
        // noreturn_lowres_msg.encoding               = sensor_msgs::image_encodings::TYPE_32FC1;
        // noreturn_lowres_msg.image                  = noreturn_lowres_img;
        // noreturn_lowres_msg.header.stamp           = ros::Time::now();
        // noreturn_lowres_msg.header.frame_id        = cloud_in_ros->header.frame_id;        
        // pub_noreturn_lowres_img_.publish(noreturn_lowres_msg.toImageMsg());

        // // Publish No Return Prob Image
        // cv_bridge::CvImage prob_noreturn_msg;
        // prob_noreturn_msg.encoding          = sensor_msgs::image_encodings::TYPE_32FC1;
        // prob_noreturn_msg.image             = prob_noreturn_img;
        // prob_noreturn_msg.header.stamp      = ros::Time::now();
        // prob_noreturn_msg.header.frame_id   = cloud_in_ros->header.frame_id;        
        // pub_prob_noreturn_img_.publish(prob_noreturn_msg.toImageMsg());

        // last_range_img          = range_img;
        // last_dev_range_img      = dev_range_img;
        // last_var_t_img          = var_t_img;
        // last_noreturn_img       = noreturn_img;
        // last_prob_noreturn_img  = prob_noreturn_img;




        // Attempt to filter non-fog out based on DoN!

        ///The smallest scale to use in the DoN filter.
        double scale1 = normal_x_LT_threshold_;

        ///The largest scale to use in the DoN filter.
        double scale2 = normal_x_GT_threshold_;

        ///The minimum DoN magnitude to threshold by
        double threshold;

        ///segment scene into clusters with given distance tolerance using euclidean clustering
        double segradius;

        // Create a search tree, use KDTreee for non-organized data.
        pcl::search::Search<pcl::PointXYZ>::Ptr tree;
        if (cloud_in2->isOrganized ())
        {
            tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
        }
        else
        {
            tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
        }

        // Set the input pointcloud for the search tree
        tree->setInputCloud (cloud_in2);

        if (scale1 >= scale2)
        {
            std::cerr << "Error: Large scale must be > small scale!" << std::endl;
            exit (EXIT_FAILURE);
        }

        // Compute normals using both small and large scales at each point
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
        ne.setInputCloud (cloud_in2);
        ne.setSearchMethod (tree);

        /**
         * NOTE: setting viewpoint is very important, so that we can ensure
         * normals are all pointed in the same direction!
         */
        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        // calculate normals with the small scale
        std::cout << "Calculating normals for scale..." << scale1 << std::endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

        ne.setRadiusSearch (scale1);
        ne.compute (*normals_small_scale);

        // calculate normals with the large scale
        std::cout << "Calculating normals for scale..." << scale2 << std::endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

        ne.setRadiusSearch (scale2);
        ne.compute (*normals_large_scale);

        // Create output cloud for DoN results
        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
        copyPointCloud (*cloud_in2, *doncloud);

        std::cout << "Calculating DoN... " << std::endl;
        // Create DoN operator
        pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
        don.setInputCloud (cloud_in2);
        don.setNormalScaleLarge (normals_large_scale);
        don.setNormalScaleSmall (normals_small_scale);

        if (!don.initCompute ())
        {
            std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
            exit (EXIT_FAILURE);
        }

        // Compute DoN
        don.computeFeature (*doncloud);


        // Filter by magnitude
        std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

        // Build the condition for filtering
        pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
            new pcl::ConditionOr<pcl::PointNormal> ()
            );
        range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                                    new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                                    );
        // Build the filter
        pcl::ConditionalRemoval<pcl::PointNormal> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (doncloud);

        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

        // Apply filter
        condrem.filter (*doncloud_filtered);

        doncloud = doncloud_filtered;

        doncloud->width = doncloud->points.size ();
        doncloud->height = 1;
        doncloud->is_dense = true;

        seq++;
        doncloud->header.seq = seq;
        doncloud->header.frame_id = cloud_in2->header.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), doncloud->header.stamp);
        pub_seg_pcl_.publish (doncloud);

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
        cv::Mat range_img = source->image(cv::Rect(x_offset, y_offset, width, height));

        // The range image is of type CV_8UC1/mono8 (0-255, 0 is no return, higher is closer to sensor)
        range_img.convertTo(range_img,CV_32FC1, 0.39215686274); // 100/255 = 0.39215686274

        cv::Mat diff_range_img = abs(range_img - last_range_img);

        last_range_img = range_img;

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
        cv::Mat new_intensity_img = source->image(cv::Rect(x_offset, y_offset, width, height));

        // The intensity image is of type CV_8UC1/mono8 (0-255)
        new_intensity_img.convertTo(new_intensity_img, CV_32FC1, 0.39215686274);

        cv::Mat diff_intensity_img = abs(new_intensity_img - last_intensity_img);

        last_intensity_img = new_intensity_img;

    };

}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( fog::FogDetectionNodelet, nodelet::Nodelet)

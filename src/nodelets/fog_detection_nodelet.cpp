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

        // sub_low_depth_pcl_ = nh.subscribe("in_pcl", 1,  &FogDetectionNodelet::point_cloud_cb, this);
        sub_low_depth_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>("in_pcl", 500, boost::bind(&FogDetectionNodelet::point_cloud_cb, this, _1));

        // Publish Point Cloud
        pub_avg_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_avg_range_img", 10);
        pub_diff_range_img_         = private_nh.advertise<sensor_msgs::Image>("out_diff_range_img", 10);
        pub_var_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_var_range_img", 10);
        pub_sum_noreturn_img_       = private_nh.advertise<sensor_msgs::Image>("out_sum_noreturn_img", 10);
        pub_dev_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_dev_range_img", 10);
        pub_dev_diff_range_img_     = private_nh.advertise<sensor_msgs::Image>("out_dev_diff_range_img", 10);
        pub_var_t_img_              = private_nh.advertise<sensor_msgs::Image>("out_var_t_img", 10);
        pub_noreturn_img_           = private_nh.advertise<sensor_msgs::Image>("out_noreturn_img", 10);
        pub_noreturn_lowres_img_    = private_nh.advertise<sensor_msgs::Image>("out_noreturn_lowres_img", 10);
        pub_prob_noreturn_img_      = private_nh.advertise<sensor_msgs::Image>("prob_noreturn_img", 10);
        pub_range_img_              = private_nh.advertise<sensor_msgs::Image>("out_range_img", 10);
        pub_intensity_img_          = private_nh.advertise<sensor_msgs::Image>("out_intensity_img", 10);
        pub_log_intensity_img_      = private_nh.advertise<sensor_msgs::Image>("out_log_intensity_img", 10);
        pub_intensity_filter_img_   = private_nh.advertise<sensor_msgs::Image>("out_intensity_filter_img", 10);
        pub_intensity_filter_pcl_   = private_nh.advertise<PointCloud>("out_intensity_filter_pcl", 10);
        pub_normal_pcl_             = private_nh.advertise<PointCloud>("out_normal_pcl", 10);

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
        
        bool range_pcl = 0;

        // Get the field structure of this point cloud
        for (int f=0; f<cloud_in_ros->fields.size(); f++)
        {
            if (cloud_in_ros->fields[f].name == "range")
            {
                range_pcl = 1;
            }
        }

        if(range_pcl == 1)
        {
            ouster_ros::OS1::CloudOS1 cloud_in{};
            pcl::fromROSMsg(*cloud_in_ros, cloud_in);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_in_ros, *cloud_in2);
            
            sensor_msgs::Image range_image;
            sensor_msgs::Image noise_image;
            sensor_msgs::Image intensity_image;

            // Get PCL metadata
            int W = cloud_in_ros->width;
            int H = cloud_in_ros->height;
            std::vector<int>  px_offset = ouster::OS1::get_px_offset(W);
            // for 64 channels, the px_offset =[ 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 
            //                                   0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18]

            // Setup Noise Image
            noise_image.width = W;
            noise_image.height = H;
            noise_image.step = W;
            noise_image.encoding = "mono8";
            noise_image.data.resize(W * H);

            // Set up Intensity Image
            intensity_image.width = W;
            intensity_image.height = H;
            intensity_image.step = W;
            intensity_image.encoding = "mono8";
            intensity_image.data.resize(W * H);

            cv::Mat img_x(H, W, CV_32FC1, 0.1);
            cv::Mat img_y(H, W, CV_32FC1, 0.1);
            cv::Mat img_z(H, W, CV_32FC1, 0.1);
            cv::Mat img_d(H, W, CV_32FC1, 0.1);
            cv::Mat img_i(H, W, CV_32FC1, 0.1);
            cv::Mat img_r(H, W, CV_32FC1, 0.1);
            cv::Mat img_n(H, W, CV_32FC1, 0.1);

            cv::Mat zero_range_img(H, W, CV_32FC1, 0.1);
            cv::Mat max_range_img(H, W, CV_32FC1, 20.0);
            cv::Mat nonzero_range_img(H, W, CV_32FC1, 0.1);
            cv::Mat range_img(H, W, CV_32FC1, 0.0);
            cv::Mat intensity_img(H, W, CV_32FC1, 0.0);
            cv::Mat log_intensity_img(H, W, CV_32FC1, 0.0);
            cv::Mat sq_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat sq_of_sum_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat zeroreturn_img(H, W, CV_32FC1, 0.0);
            cv::Mat acc_noreturn_msk(H, W, CV_32FC1, 0.0);
            cv::Mat avg_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat diff_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat var_s_raw_img(H, W, CV_32FC1, 0.0);
            cv::Mat var_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_range_bin_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_diff_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat var_t_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_sum_img(H, W, CV_32FC1, 0.0);
            cv::Mat noreturn_img(H, W, CV_32FC1, 0.0);
            cv::Mat prob_noreturn_img(H, W, CV_32FC1, 0.0);

            cv::Mat zero_range_msk(H, W, CV_32FC1, 0.0);

            cv::Mat intensity_filter_img(H, W, CV_32FC1, 0.0);

            float dist = 0;
            float max_dist = 0;
            float last_dist = 0;

            for (int u = 0; u < H; u++) {
                for (int v = 0; v < W; v++) {
                    const size_t vv = (v + px_offset[u]) % W;
                    const size_t index = vv * H + u;
                    const auto& pt = cloud_in[index];
                    range_img.at<float>(u,v) = pt.range * 1e-3; // Physical range from 0 - 100 m (converting from mm --> m)
                    // range_img.at<float>(u,v) = pt.range * 5e-3; // Range for 8-bit image from 0 - 255
                    noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);
                    intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);
                    intensity_img.at<float>(u,v) = pt.intensity + 1.0f;
                }
            }

            cv::threshold(intensity_img, intensity_filter_img, 100.0, 1.0, THRESH_BINARY_INV);

            // std::cout << intensity_filter_img << std::endl;

            // Plane fitting
            // int window_sz = 3;
            // int ofst = (window_sz - 1) / 2;

            // for (int u = 0 + ofst; u < H - ofst; u++) {
            //     for (int v = 0 + ofst; v < W - ofst; v++) {
            //         const size_t vv = (v + px_offset[u]) % W;
            //         const size_t index = vv * H + u;
            //         const auto& pt = cloud_in[index];
            //         range_img.at<float>(u,v) = pt.range * 1e-3; // Physical range from 0 - 100 m (converting from mm --> m)
            //         // range_img.at<float>(u,v) = pt.range * 5e-3; // Range for 8-bit image from 0 - 255
            //         noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);
            //         intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);
            //         intensity_img.at<float>(u,v) = pt.intensity + 1.0f;
            //     }
            // }

            // Intensity Filter
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ExtractIndices<pcl::PointXYZI> extract;

            for (int u = 0; u < H; u++)
            {
                for (int v = 0; v < W; v++)
                {
                    if(intensity_filter_img.at<float>(u,v) > 0.1)
                    {
                        const size_t vv = (v + px_offset[u]) % W;
                        const size_t i = vv * H + u;
                        inliers->indices.push_back(i);
                    }
                }
            }

            extract.setInputCloud(cloud_in2);
            extract.setIndices(inliers);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*output);

            seq++;
      
            output->width = output->points.size ();
            output->height = 1;
            output->is_dense = true;
            output->header.seq = seq;
            output->header.frame_id = cloud_in2->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);
            pub_intensity_filter_pcl_.publish (output);

            // Publish Range Image
            cv_bridge::CvImage new_range_msg;
            new_range_msg.encoding                  = sensor_msgs::image_encodings::TYPE_32FC1;
            new_range_msg.image                     = range_img;
            new_range_msg.header.stamp              = ros::Time::now();
            new_range_msg.header.frame_id           = cloud_in_ros->header.frame_id;        
            pub_range_img_.publish(new_range_msg.toImageMsg());

            // Publish Intensity Image
            cv_bridge::CvImage intensity_msg;
            intensity_msg.encoding              = sensor_msgs::image_encodings::TYPE_32FC1;
            intensity_msg.image                 = intensity_img;
            intensity_msg.header.stamp          = ros::Time::now();
            intensity_msg.header.frame_id       = cloud_in_ros->header.frame_id;        
            pub_intensity_img_.publish(intensity_msg.toImageMsg());

            // Publish Intensity Filter
            cv_bridge::CvImage intensity_filter_msg;
            intensity_filter_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
            intensity_filter_msg.image                      = intensity_filter_img;
            intensity_filter_msg.header.stamp               = ros::Time::now();
            intensity_filter_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
            pub_intensity_filter_img_.publish(intensity_filter_msg.toImageMsg());

            ///////////////////////
            // CALCULATE NORMALS //
            ///////////////////////

            bool flag_pub_normal_pcl = 0;
            
            if(flag_pub_normal_pcl)
            {

                pcl::search::Search<pcl::PointXYZI>::Ptr tree_xyz;

                // Use KDTree for non-organized data
                tree_xyz.reset(new pcl::search::KdTree<pcl::PointXYZI> (false));

                // if (cloud_in2->isOrganized ())
                // {
                //     std::cout << "org" << std::endl;
                //     tree_xyz.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZI> ());
                // }
                // else
                // {
                //     std::cout << "not org" << std::endl;
                //     // Use KDTree for non-organized data
                //     tree_xyz.reset(new pcl::search::KdTree<pcl::PointXYZI> (false));
                // }

                // Set input pointcloud for search tree
                tree_xyz->setInputCloud(cloud_in2);

                pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointXYZINormal> ne; // NormalEstimationOMP uses more CPU on NUC (do not use OMP!)
                ne.setInputCloud(cloud_in2);
                ne.setSearchMethod(tree_xyz);

                // Set viewpoint, very important so normals are all pointed in the same direction
                ne.setViewPoint(std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ne (new pcl::PointCloud<pcl::PointXYZINormal>);

                ne.setRadiusSearch(0.05);
                ne.compute(*cloud_ne);

                // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)
                for (int i = 0; i < cloud_in2->points.size(); i++)
                {
                    cloud_ne->points[i].x = cloud_in2->points[i].x;
                    cloud_ne->points[i].y = cloud_in2->points[i].y;
                    cloud_ne->points[i].z = cloud_in2->points[i].z;
                    cloud_ne->points[i].intensity = cloud_in2->points[i].intensity;
                }
                
                cloud_ne->width = cloud_ne->points.size ();
                cloud_ne->height = 1;
                cloud_ne->is_dense = true;
                cloud_ne->header.seq = seq;
                cloud_ne->header.frame_id = cloud_in2->header.frame_id;
                pcl_conversions::toPCL(ros::Time::now(), cloud_ne->header.stamp);
                pub_normal_pcl_.publish (cloud_ne);

            }



            
        }
    
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

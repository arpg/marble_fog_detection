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
        pub_conf_pcl_               = private_nh.advertise<PointCloud>("out_conf_pcl", 10);
        pub_avg_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_avg_range_img", 10);
        pub_sum_noreturn_img_       = private_nh.advertise<sensor_msgs::Image>("out_sum_noreturn_img", 10);
        pub_dev_range_img_          = private_nh.advertise<sensor_msgs::Image>("out_dev_range_img", 10);
        pub_noreturn_lowres_img_    = private_nh.advertise<sensor_msgs::Image>("out_noreturn_lowres_img", 10);
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

        int H = 64;
        int W = 1024;
        
        azim_LUT.resize(W);
        float azim_res = ((180) - (-180) ) / W;
        linearSpacedArray(azim_LUT, -180, 180 - azim_res, W);

        std::cout << "The azimuthal angles are: ";
        for(int i=0; i < azim_LUT.size(); i++)
        std::cout << azim_LUT.at(i) << ' ';
        std::cout << std::endl;
        std::cout << std::endl;
        
        elev_LUT.resize(H);
        linearSpacedArray(elev_LUT, -16.611, 16.611, H - 1);

        std::cout << "The elevation angles are: ";
        for(int i=0; i < elev_LUT.size(); i++)
        std::cout << elev_LUT.at(i) << ' ';
        std::cout << std::endl;
        std::cout << std::endl;
                
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
        ror_radius_                 = config.ror_radius;
        ror_min_neighbors_          = config.ror_min_neighbors;
        fog_min_range_deviation_    = config.fog_min_range_deviation;
        fog_radius_high_intensity_  = config.fog_radius_high_intensity;
        
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

        // // Note roll and pitch are intentionally backwards due to the image frame to boldy frame transform_pcl. The IMU transform_pcl converts from body to world frame.
        // tf::Transform transform_pcl;
        // transform_pcl.setRotation(tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_));
        // pcl_ros::transformPointCloud(*cloud_in_filtered, *cloud_in_transformed, transform_pcl);

        // tf::TransformListener listener;
        // tf::StampedTransform T_map_os1lidar;
        // double roll, pitch, yaw;
        
        // try
        // {
        //     listener.waitForTransform("/os1_lidar", "/map", ros::Time::now(), ros::Duration(0.05) );
        //     listener.lookupTransform("/os1_lidar", "/map", ros::Time(0), T_map_os1lidar);
        //     tf::Matrix3x3 m(T_map_os1lidar.getRotation());
        //     int solution_number = 1;
        //     m.getEulerYPR(yaw, pitch, roll, solution_number);
        // }
        // catch (tf::TransformException ex)
        // {
        //     ROS_WARN("%s",ex.what()); 
        //     // ros::Duration(0.1).sleep();
        // }
        
        bool range_pcl = 0;

        // Get the field structure of this point cloud
        for (int f=0; f<cloud_in_ros->fields.size(); f++)
        {
            if (cloud_in_ros->fields[f].name == "range")
            {
                range_pcl = 1;
            }
        }

        if(range_pcl == 0)
        {


            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_in_ros, *cloud_in2);
            

            // X: NORTH
            // Y: EAST
            // Z: UP
            
            // NOTE: XYZ --> NWU (ouster_link frame), indices start at NORTH and rotate CLOCKWISE
            // NOTE: Azim angles start at NORTH and rotate CLOCKWISE (in ouster_link frame)
            // NOTE: Azim angles are -180 (NORTH), ... -90 (EAST), ... 0 (SOUTH), 90 (WEST), ... 180 (NORTH)

            std::vector<double> range(cloud_in2->points.size());
            std::vector<double> azim_angle(cloud_in2->points.size());
            std::vector<double> elev_angle(cloud_in2->points.size());

            int H = 64;
            int W = 1024;
            cv::Mat range_img(H, W, CV_32FC1, 0.0);

            int u;
            int v;

            for (int i = 0; i < cloud_in2->points.size(); i++)
            {
                const auto& pt      = cloud_in2->points[i];
                float x2            = pt.x * pt.x;
                float y2            = pt.y * pt.y;
                float z2            = pt.z * pt.z;
                range[i]            = sqrt(x2 + y2 + z2);
                if(range[i] > 0)
                {
                    azim_angle[i]   = atan2(pt.y, -pt.x) * 180 / M_PI; // THIS IS CURRENTLY WRONG I BELIEVE
                    if(azim_angle[i] >= M_PI - 0.01)
                    {
                        azim_angle[i] = azim_angle[i] - 2*M_PI;
                    }

                    elev_angle[i]   = atan2(pt.z, sqrt(x2 + y2)) * 180 / M_PI;

                    u = (H-1) - binarySearch(elev_LUT, elev_angle[i], 0.528/2.0);
                    // std::cout << elev_angle[i] << " -> " << u << std::endl;
                    
                    v = binarySearch(azim_LUT, azim_angle[i], 0.36/2.0);
                    if(v > (W-1))
                    {
                        std::cout << "v: " << v << std::endl;
                        std::cout << "azim_angle: " << azim_angle[i] << std::endl;
                    }
                    // std::cout << azim_angle[i] << " -> " << v << std::endl;

                range_img.at<float>(u,v) = range[i]; // Physical range from 0 - 100 m (converting from mm --> m)
                }
                else
                {
                    azim_angle[i]   = 0;
                    elev_angle[i]   = 0;
                }

                // std::cout << "Point " << i << " / X " << pt.x << " / Y " << pt.y << " / Z " << pt.z << " / Range " << range[i] << " / Azim Angle " << azim_angle[i] <<  " / Elev Angle "  << elev_angle[i] << std::endl;
                
            }

            // Publish Range Image
            cv_bridge::CvImage new_range_msg;
            new_range_msg.encoding                  = sensor_msgs::image_encodings::TYPE_32FC1;
            new_range_msg.image                     = range_img;
            new_range_msg.header.stamp              = ros::Time::now();
            new_range_msg.header.frame_id           = cloud_in_ros->header.frame_id;        
            pub_range_img_.publish(new_range_msg.toImageMsg());
            
            seq++;

            // std::cout << "The azimuthal angles are: ";
            // for(int i=0; i < azim_LUT.size(); i++)
            // std::cout << azim_LUT.at(i) << ' ';
            // std::cout << std::endl;
            // std::cout << std::endl;

            // std::cout << "The elevation angles are: ";
            // for(int i=0; i < elev_LUT.size(); i++)
            // std::cout << elev_LUT.at(i) << ' ';
            // std::cout << std::endl;
            // std::cout << std::endl;

            // std::cout << std::endl;
            // std::cout << std::endl;

            // std::cout << "---------BEGIN LOOP----------" << std::endl;

            // std::cout << "The azimuthal angles are : " << std::endl;
            // for(int i=0; i < azim_angle.size(); i++)
            // {
            //     std::cout << azim_angle.at(i) << ' ';
            // };

            // std::cout << "The elevation angles are : " << std::endl;
            // for(int i=0; i < elev_angle.size(); i++)
            // {
            //     std::cout << elev_angle.at(i) << ' ';
            // };

            // std::cout << std::endl;
            // std::cout << std::endl;


            // std::vector<double> azim_LUT = {0.0, 50.0, 89.0, 100.0};
            // double ans;

            // ans = binarySearch(azim_LUT, 86.0, 2.0);
            // std::cout << "86 -> " << ans << std::endl;

            // ans = binarySearch(azim_LUT, 87.0, 2.0);
            // std::cout << "87 -> " << ans << std::endl;

            // ans = binarySearch(azim_LUT, 88.0, 2.0);
            // std::cout << "88 -> " << ans << std::endl;

            // ans = binarySearch(azim_LUT, 89.0, 2.0);
            // std::cout << "89 -> " << ans << std::endl;

            // ans = binarySearch(azim_LUT, 90.0, 2.0);
            // std::cout << "90 -> " << ans << std::endl;
            
            // ans = binarySearch(azim_LUT, 91.0, 2.0);
            // std::cout << "91 -> " << ans << std::endl;

            // ans = binarySearch(azim_LUT, 92.0, 2.0);
            // std::cout << "92 -> " << ans << std::endl;








            // pcl::PointCloud<pcl::PointXYZI>::Ptr few_points(new pcl::PointCloud<pcl::PointXYZI>);

            // extract.setInputCloud(cloud_in2);
            // extract.setIndices(inliers);
            // extract.setNegative(false);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr single_point(new pcl::PointCloud<pcl::PointXYZI>);
            // extract.filter(*single_point);
            
            // single_point->width = single_point->points.size ();
            // single_point->height = 1;
            // single_point->is_dense = true;

            // seq++;
            // single_point->header.seq = seq;
            // single_point->header.frame_id = cloud_in2->header.frame_id;
            // pcl_conversions::toPCL(ros::Time::now(), single_point->header.stamp);
            // pub_conf_pcl_.publish (single_point);

            // few_points->width = single_point->points.size ();
            // few_points->height = 1;
            // few_points->is_dense = true;

            // seq++;
            // few_points->header.seq = seq;
            // few_points->header.frame_id = cloud_in2->header.frame_id;
            // pcl_conversions::toPCL(ros::Time::now(), few_points->header.stamp);
            // pub_conf_pcl_.publish (few_points);



            // // // Try to publish half of the point cloud
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            // pcl::ExtractIndices<pcl::PointXYZI> extract;

            // for (int i = seq * 16; i < seq * 16 + 16; i++)
            // {
            //     inliers->indices.push_back(i);
            // }

            // extract.setInputCloud(cloud_in2);
            // extract.setIndices(inliers);
            // extract.setNegative(false);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
            // extract.filter(*output);
            
            // output->width = output->points.size ();
            // output->height = 1;
            // output->is_dense = true;

            // seq++;
            // output->header.seq = seq;
            // output->header.frame_id = cloud_in2->header.frame_id;
            // pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);
            // pub_conf_pcl_.publish (output);

        }

        if(range_pcl == 1)
        {
            // start 0.038118956 seconds
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

            cv::Mat zero_range_img(H, W, CV_32FC1, 0.1);
            cv::Mat max_range_img(H, W, CV_32FC1, 20.0);
            cv::Mat nonzero_range_img(H, W, CV_32FC1, 0.1);
            cv::Mat range_img(H, W, CV_32FC1, 0.0);
            cv::Mat x_img(H, W, CV_32FC1, 0.0);
            cv::Mat y_img(H, W, CV_32FC1, 0.0);
            cv::Mat z_img(H, W, CV_32FC1, 0.0);
            cv::Mat avg_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat diff_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat var_s_raw_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_range_bin_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_sum_img(H, W, CV_32FC1, 0.0);
            cv::Mat noreturn_img(H, W, CV_32FC1, 0.0);

            cv::Mat zero_range_msk(H, W, CV_32FC1, 0.0);

            float dist = 0;
            float max_dist = 0;
            float last_dist = 0;

            // NOTE: XYZ --> NWU (os1_lidar frame), indices start at NORTH and rotate CLOCKWISE
            // NOTE: Azim angle should be -180 ... 0 ... 180

            // NOTE: AZIMUTHAL ANGLES IN EACH COLUMN
            // NOTE:    COLUMN 0, COLUMN 1, ..., COLUMN 502, COLUMN 503, COLUMN 504, COLUMN 505, ..., COLUMN 1022, COLUMN 1023
            // NOTE:    -3.113,   -3.46456, ..., -179.59612, -179.94768,  179.70076,   179.3492, ...,    -2.40732,    -2.75888
            for (int u = 0; u < H; u++) {
                for (int v = 0; v < W; v++) {
                    const size_t vv = (v + px_offset[u]) % W;
                    const size_t index = vv * H + u;
                    const auto& pt = cloud_in[index];
                    x_img.at<float>(u,v) = pt.x; // Physical x-coordinate (NORTH)
                    y_img.at<float>(u,v) = pt.y; // Physical y-coordinate (WEST)
                    z_img.at<float>(u,v) = pt.z; // Physical z-coordinate (UP)
                    range_img.at<float>(u,v) = pt.range * 1e-3; // Physical range from 0 - 100 m (converting from mm --> m)
                    // range_img.at<float>(u,v) = pt.range * 5e-3; // Range for 8-bit image from 0 - 255
                    noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);
                    intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);
                }
            }

            std::cout << "------------------------ BEGIN LOOP ------------------------" << std::endl;

            float x_val;
            float y_val;
            std::vector<double> azim_val(W);
            std::vector<double> elev_val(H);

            for (int i = 0; i < W; i++)
            {
                azim_val[i] = atan2(y_img.at<float>(0,i), -x_img.at<float>(0,i)) * 180 / M_PI;
            }
            for (int i = 0; i < H; i++)
            {
                elev_val[i] = atan2(z_img.at<float>(i,40), sqrt(x_img.at<float>(i,40) * x_img.at<float>(i,40) 
                                                              + y_img.at<float>(i,40) * y_img.at<float>(i,40))) * 180 / M_PI;
            } 
            
            std::cout << "The azimuthal angles are : " << std::endl;
            for(int i=0; i < azim_val.size(); i++)
            {
                std::cout << azim_val.at(i) << ' ';
            };

            std::cout << "Point " << 10 << " / X " << x_img.at<float>(10,40) << " / Y " << y_img.at<float>(10,40) << " / Z " << "Azim Angle " << azim_val[9] <<  " / Elev Angle "  << elev_val[39] << std::endl;

            // std::cout << "The elevation angles are : " << std::endl;
            // for(int i=0; i < elev_val.size(); i++)
            // {
            //     std::cout << elev_val.at(i) << ' ';
            // };

            // std::cout << std::endl;

            // stop 0.038118956 seconds
            // start 0.002747821 seconds

            if(last_range_img.rows == 0 && last_range_img.cols == 0)
            {
            }
            else
            {
                               
                // Binarize depth (range) image
                cv::threshold(range_img, return_img, 0.0, 1.0, THRESH_BINARY);

                // cv::blur(range_img, mean_s_img, cv::Size(9,9)); //Or whatever blurring you want

                // Average range image
                cv::boxFilter(range_img, avg_range_img, -1, cv::Size(5,5), cv::Point(-1,-1), false); // do not normalize here

                // Average binarized range image (return image)
                cv::boxFilter(return_img, return_sum_img, -1, cv::Size(5,5), cv::Point(-1,-1), false); // do not normalize here

                // Scale average range image
                cv::divide(avg_range_img, return_sum_img, avg_range_img); // normalize here

                // Subtract range image from average range image, threshold betwneen 0 and 10
                cv::threshold(avg_range_img - range_img, dev_range_img, 0.0, 0.0, THRESH_TOZERO);
                cv::threshold(dev_range_img, dev_range_img, 10.0, 10.0, THRESH_TRUNC);

                // Subtract range image from average range image, binarize as pre-filter
                cv::threshold(avg_range_img - range_img, dev_range_bin_img, 0.10, 1.0, THRESH_BINARY);

                // Compute binary no-return image (1 = no return, 0 = return)
                cv::threshold(range_img, noreturn_img, 0.2, 1.0, THRESH_BINARY_INV);
                
                double min = 0, max = 0;
                cv::Point minLoc(-1, -1), maxLoc(-1, -1);
                // https://stackoverflow.com/questions/18233691/how-to-index-and-modify-an-opencv-matrix

            }

            // Publish Range Image
            cv_bridge::CvImage new_range_msg;
            new_range_msg.encoding                  = sensor_msgs::image_encodings::TYPE_32FC1;
            new_range_msg.image                     = range_img;
            new_range_msg.header.stamp              = ros::Time::now();
            new_range_msg.header.frame_id           = cloud_in_ros->header.frame_id;        
            pub_range_img_.publish(new_range_msg.toImageMsg());

            // Publish Range Deviation Image
            cv_bridge::CvImage dev_range_msg;
            dev_range_msg.encoding                   = sensor_msgs::image_encodings::TYPE_32FC1;
            dev_range_msg.image                      = dev_range_img;
            dev_range_msg.header.stamp               = ros::Time::now();
            dev_range_msg.header.frame_id            = cloud_in_ros->header.frame_id;        
            pub_dev_range_img_.publish(dev_range_msg.toImageMsg());
            
            last_range_img          = range_img;

            // Image Filter

            // compute sum of positive matrix elements
            // (assuming that M isa double-precision matrix)
            double sum=0;
            int n_above = 2;
            int n_below = 2;
            int n_left  = 2;
            int n_right = 2;

            cv::Mat filtered_img(H, W, CV_32FC1, 0.0);

            std::vector<float> kernel_vec = {1/sqrt(2) , 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                            1/sqrt(2) , 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                            1/sqrt(2) , 1/sqrt(2),         0, 1/sqrt(2), 1/sqrt(2),
                                            1/sqrt(2) , 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                            1/sqrt(2) , 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2)};
            
            std::vector<float> bool_vec;

            // end 0.002747821 seconds
            // start 0.02977038 seconds

            // Iterate over each pixel
            for(int i = 0; i < range_img.rows; i++)
            {
                for(int j = 0; j < range_img.cols; j++)
                {
                    // Ignore border pixels, ignore pixels with current or previous value of 0 (no return) 
                    if( (i < n_above) || (i > range_img.rows - n_below) || (j < n_left) || (j > range_img.cols - n_right || range_img.at<float>(i,j) == 0) || dev_range_bin_img.at<float>(i,j) == 0 )
                    {
                        filtered_img.at<float>(i,j) = 0;
                    }
                    else
                    {
                        std::vector<float> data_vec;
                        std::vector<float> pixel_vec(25, range_img.at<float>(i,j));

                        // Add a neighboring box of 25 pixels (5x5) to an vector
                        for(int m = -n_left; m <= n_right; m++)
                        {
                            for(int n = -n_above; n <= n_below; n++)
                            {
                                data_vec.push_back(range_img.at<float>(i-m,j-n));
                            }
                        }

                        // std::cout << "data_vec: ";
                        // for (auto i : data_vec) std::cout << i << ' ';
                        // std::cout << '\n';

                        for(auto& element : data_vec)
                        {
                            if(element != 0)
                            {
                                // Compute the normalized deviation from the center pixel range: (pixel - center pixel)/center pixel
                                // Threshold so only background points are ignored (only keep foreground points)
                                element = std::max(-0.01f, (element - range_img.at<float>(i,j)))/element;
                            }
                        }

                        // Sum the normalized deviation
                        float sum = std::inner_product(std::begin(data_vec), std::end(data_vec), std::begin(kernel_vec), 0.0);

                        float cnt_nonzero = std::count_if(data_vec.begin(), data_vec.end(),[&](float const& val){ return val != 0; });

                        // Divide the sum of normalized deviation by the number of value returns in the 5x5 box (ignore 0’s)
                        float avgval = sum / cnt_nonzero;

                        filtered_img.at<float>(i,j) = avgval;
                        
                        // std::cout << "data_vec: ";
                        // for (auto i : data_vec) std::cout << i << ' ';
                        // std::cout << '\n';

                        // std::cout << "pixel_vec: ";
                        // for (auto i : pixel_vec) std::cout << i << ' ';
                        // std::cout << '\n';

                        // std::cout << "kernel_vec: ";
                        // for (auto i : kernel_vec) std::cout << i << ' ';
                        // std::cout << '\n';

                        // std::cout << "dot_product: " << sum;
                        // std::cout << '\n';

                        // std::cout << "cnt_nonzero: " << cnt_nonzero;
                        // std::cout << '\n';

                        // std::cout << "avgval: " << avgval;
                        // std::cout << '\n';

                        // std::cout << '\n';
                    }
                }
            }

            // end 0.02977038 seconds
            // start 0.025598312 seconds

            // std::cout << "filtered_img: " << filtered_img << std::endl;

            // Extract PCL Indices
            cv::Mat filter_img(H, W, CV_32FC1, 0.0);
            cv::threshold(filtered_img, filter_img, fog_min_range_deviation_, 1.0, THRESH_TOZERO);
            
            // // Try to publish half of the point cloud
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ExtractIndices<pcl::PointXYZI> extract;

            // https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/kdtree_search.html
            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud (cloud_in2);
            pcl::PointXYZI search_pt;
            bool flag_fog;

            // end 0.025598312 seconds            
            // start 0.004650553 seconds

            // Iterate through the depth image
            for (int u = 0; u < H; u++)
            {
                for (int v = 0; v < W; v++)
                {
                    // If the pixel is 0.1m closer than the surrounding backgroud pixels, investigate:
                    if(filter_img.at<float>(u,v) > 0.1)
                    {
                        const size_t vv = (v + px_offset[u]) % W;
                        const size_t i = vv * H + u;
                        search_pt = cloud_in2->points[i];

                        // If the point is withint the intensity band of fog, then check out it's neighbors
                        if(search_pt.intensity > 10 && search_pt.intensity < 100)
                        {
                            // Neighbors within radius search
                            std::vector<int> pointIdxRadiusSearch;
                            std::vector<float> pointRadiusSquaredDistance;
                            
                            // If it has neighbors
                            if (kdtree.radiusSearch (search_pt, fog_radius_high_intensity_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                            {
                                // Iterate over neighbors, and disregard if its neighbors have high-intensity points
                                flag_fog = 1;
                                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                                {
                                    if(cloud_in2->points[ pointIdxRadiusSearch[i] ].intensity > 100)
                                    {
                                        flag_fog = 0;
                                    }
                                }
                                // If all neighbors are in intensity band of fog, then include this point in the fog pcl
                                if(flag_fog == 1)
                                {
                                    inliers->indices.push_back(i);
                                }
                            }

                        }

                    }
                }
            }

            // end 0.004650553 seconds
            // start 0.000119647 seconds
            
            extract.setInputCloud(cloud_in2);
            extract.setIndices(inliers);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*output);
            
            output->width = output->points.size ();
            output->height = 1;
            output->is_dense = true;

            seq++;
            output->header.seq = seq;
            output->header.frame_id = cloud_in2->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);
            pub_conf_pcl_.publish (output);


            // end 0.000119647 seconds

            // auto start = high_resolution_clock::now();
            // auto stop = high_resolution_clock::now();

            // // Subtract stop and start timepoints and
            // // cast it to required unit. Predefined units
            // // are nanoseconds, microseconds, milliseconds,
            // // seconds, minutes, hours. Use duration_cast()
            // // function.

            // auto duration = duration_cast<nanoseconds>(stop - start);

            // // To get the value of duration use the count()
            // // member function on the duration object
            // std::cout << "\n" << duration.count() << "\n" << std::endl;

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

    double FogDetectionNodelet::binarySearch(std::vector<double>& array, const double value, const double threshold) 
    {
        // I assume here that the array is sorted ...
        // If I cannot find it, I will return infinity (:

        double returnValue = std::numeric_limits<double>::infinity(); // if you want to return the LUT value

        std::vector<double>::iterator it = std::lower_bound(array.begin(), array.end(), value - threshold);

        if(it != array.end()) 
        {
            if(fabs(*it - value) <= threshold ) returnValue = *it;
        }
        
        auto returnIndex = std::distance(array.begin(), it); // if you want to return the LUT index

        return returnIndex;
    }

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::linearSpacedArray(std::vector<double> &xs, double a, double b, std::size_t N)
    {
        double h = (b - a) / static_cast<double>(N);
        std::vector<double>::iterator x;
        double val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        {
            *x = val;
        }
    } 



}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( fog::FogDetectionNodelet, nodelet::Nodelet)

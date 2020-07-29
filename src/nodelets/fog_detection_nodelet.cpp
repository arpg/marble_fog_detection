#include <fog_detection/fog_detection.h>
#include <chrono>

namespace fog
{
    void FogDetectionNodelet::connectCb()
    {
    };

    void FogDetectionNodelet::onInit()
    {
        nh         = getMTNodeHandle();
        private_nh = getMTPrivateNodeHandle();

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("target_frame_id", target_frame_id_, std::string());

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&FogDetectionNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

        // Wait for first message to be published, determine format of incoming data
        boost::shared_ptr<sensor_msgs::PointCloud2 const> init_cloud_in_ros;
        sensor_msgs::PointCloud2 edge;
        init_cloud_in_ros = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("in_pcl", nh);
        if(init_cloud_in_ros != NULL)
        {
            // Get the field structure of this point cloud
            for (int f=0; f<init_cloud_in_ros->fields.size(); f++)
            {
                if (init_cloud_in_ros->fields[f].name == "range")
                {
                    flag_official_ouster_pcl = 1;
                    H = init_cloud_in_ros->height;
                    W = init_cloud_in_ros->width;
                }
                else
                {
                    flag_official_ouster_pcl = 0;
                    H = 64;
                    W = 1024;

                }
                
            }
            
            px_offset.resize(H);
            px_offset = ouster::OS1::get_px_offset(W);
            // for 64 channels, the px_offset =[ 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 
            //                                   0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18]

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
            
        }

        // https://stackoverflow.com/questions/48497670/multithreading-behaviour-with-ros-asyncspinner/48544551
        // It is possible to allow concurrent calls by setting the correct 
        // SubscribeOptions.allow_concurrent_callbacks which is false by default
        // Therefore you need to define your own SubscribeOptions.
        // Here is the code you need to subscribe and allow concurrent callback calls:
        ros::SubscribeOptions ops;
        ops.template init<sensor_msgs::PointCloud2>("in_pcl", 10, boost::bind(&FogDetectionNodelet::point_cloud_cb, this, _1));
        ops.transport_hints             = ros::TransportHints();
        ops.allow_concurrent_callbacks  = true;
        sub_pcl_                        = nh.subscribe(ops);

        // sub_pcl_                    = nh.subscribe<sensor_msgs::PointCloud2>("in_pcl", 500, boost::bind(&FogDetectionNodelet::point_cloud_cb, this, _1));

        // Publish Point Cloud
        pub_foreground_pcl_         = private_nh.advertise<sensor_msgs::PointCloud2>("out_foreground_pcl", 10);
        pub_foreground_z_pcl_       = private_nh.advertise<sensor_msgs::PointCloud2>("out_foreground_z_pcl", 10);
        pub_fog_pcl_                = private_nh.advertise<sensor_msgs::PointCloud2>("out_fog_pcl", 10);
        pub_range_img_              = private_nh.advertise<sensor_msgs::Image>("out_range_img", 10);
        pub_intensity_img_          = private_nh.advertise<sensor_msgs::Image>("out_intensity_img", 10);

    };

    void FogDetectionNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;

        fog_min_range_deviation_        = config.fog_min_range_deviation;
        fog_radius_high_intensity_      = config.fog_radius_high_intensity;
        fog_low_intensity_              = config.fog_low_intensity;
        fog_high_intensity_             = config.fog_high_intensity;
        
    };

    void FogDetectionNodelet::point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_ros)
    {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

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

        if(flag_official_ouster_pcl == 0)
        {

            // START 0.010 seconds
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_in_ros, *cloud_in2);

            // END 0.010 seconds

            cv::Mat range_img(H, W, CV_32FC1, 0.0);
            cv::Mat index_img(H, W, CV_32FC1, 0.0);
            cv::Mat filter_img(H, W, CV_32FC1, 0.0);

            // START 0.050 seconds

            getDepthImageCPFL(cloud_in2, range_img, index_img);

            // END 0.050 seconds

            // START 0.012 seconds

            getPreFilterImage(range_img, filter_img);

            // END 0.012 seconds

            // START 0.016 seconds

            // Extract PCL Indices // https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/kdtree_search.html
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud (cloud_in2);
            pcl::PointXYZI search_pt;
            bool flag_fog;

            // END 0.016 seconds

            // START 0.003 seconds

            int ii = 0;
            // Iterate through the depth image
            for (int u = 0; u < H; u++)
            {
                for (int v = 0; v < W; v++)
                {
                    // If the pixel is 0.1m closer than the surrounding backgroud pixels, investigate:
                    if(filter_img.at<float>(u,v) > 0.1)
                    {
                        ii++;
                        const size_t i = index_img.at<float>(u,v);
                        search_pt = cloud_in2->points[i];
                        labelForegroundFilterPCL(search_pt, cloud_in2, inliers, kdtree, i);
                        // std::cout << "accum count: " << ii << std::endl;
                        // std::cout << "inliers->indices.size(): " << inliers->indices.size() << std::endl;
                    }
                }
            }
            
            // END 0.003 seconds
            
            getPostFilterPcl(cloud_in2, inliers);

        }

        if(flag_official_ouster_pcl == 1)
        {
            // start 0.038118956 seconds
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud_in_ros, *cloud_in2);
            
            sensor_msgs::Image range_image;
            sensor_msgs::Image noise_image;
            sensor_msgs::Image intensity_image;
            
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
            cv::Mat range_img(H, W, CV_32FC1, 0.0);
            cv::Mat x_img(H, W, CV_32FC1, 0.0);
            cv::Mat y_img(H, W, CV_32FC1, 0.0);
            cv::Mat z_img(H, W, CV_32FC1, 0.0);
            cv::Mat diff_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat dev_range_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_img(H, W, CV_32FC1, 0.0);
            cv::Mat return_sum_img(H, W, CV_32FC1, 0.0);

            cv::Mat zero_range_msk(H, W, CV_32FC1, 0.0);

            cv::Mat filter_img(H, W, CV_32FC1, 0.0);

            getDepthImageOfficial(cloud_in_ros, range_img);


            // DEBUG: Print azim and elev angles of each point

            // float x_val;
            // float y_val;
            // std::vector<double> azim_val(W);
            // std::vector<double> elev_val(H);

            // for (int i = 0; i < W; i++)
            // {
            //     azim_val[i] = atan2(y_img.at<float>(0,i), -x_img.at<float>(0,i)) * 180 / M_PI;
            // }
            // for (int i = 0; i < H; i++)
            // {
            //     elev_val[i] = atan2(z_img.at<float>(i,40), sqrt(x_img.at<float>(i,40) * x_img.at<float>(i,40) 
            //                                                   + y_img.at<float>(i,40) * y_img.at<float>(i,40))) * 180 / M_PI;
            // } 
            
            // std::cout << "The azimuthal angles are : " << std::endl;
            // for(int i=0; i < azim_val.size(); i++)
            // {
            //     std::cout << azim_val.at(i) << ' ';
            // };

            // std::cout << "Point " << 10 << " / X " << x_img.at<float>(10,40) << " / Y " << y_img.at<float>(10,40) << " / Z " << "Azim Angle " << azim_val[9] <<  " / Elev Angle "  << elev_val[39] << std::endl;

            // std::cout << "The elevation angles are : " << std::endl;
            // for(int i=0; i < elev_val.size(); i++)
            // {
            //     std::cout << elev_val.at(i) << ' ';
            // };

            // std::cout << std::endl;

            // stop 0.038118956 seconds
            // start 0.002747821 seconds

            // Skip if first frame
            if(last_range_img.size().height > 0)
            {
                getPreFilterImage(range_img, filter_img);
            }
            
            last_range_img = range_img;


            // end 0.02977038 seconds
            // start 0.025598312 seconds

            // Extract PCL Indices
            
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
                        labelForegroundFilterPCL(search_pt, cloud_in2, inliers, kdtree, i);
                    }
                }
            }

            // end 0.004650553 seconds
            
            getPostFilterPcl(cloud_in2, inliers);
            
            // end 0.000119647 seconds
            
            // Timing Code
            // https://stackoverflow.com/questions/2808398/easily-measure-elapsed-time
        }

        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << " [ns]" << std::endl << std::endl;
            
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

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::getDepthImageCPFL(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2, cv::Mat &range_img, cv::Mat &index_img)
    {
        
        // X: NORTH
        // Y: EAST
        // Z: UP
        
        // NOTE: XYZ --> NWU (ouster_link frame), indices start at NORTH and rotate CLOCKWISE
        // NOTE: Azim angles start at NORTH and rotate CLOCKWISE (in ouster_link frame)
        // NOTE: Azim angles are -180 (NORTH), ... -90 (EAST), ... 0 (SOUTH), 90 (WEST), ... 180 (NORTH)

        std::vector<double> range(cloud_in2->points.size());
        std::vector<double> azim_angle(cloud_in2->points.size());
        std::vector<double> elev_angle(cloud_in2->points.size());

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
                azim_angle[i]   = atan2(pt.y, -pt.x) * 180 / M_PI;
                if(azim_angle[i] >= M_PI - 0.01)
                {
                    azim_angle[i] = azim_angle[i] - 2*M_PI;
                }

                elev_angle[i]   = atan2(pt.z, sqrt(x2 + y2)) * 180 / M_PI;

                u = (64-1) - binarySearch(elev_LUT, elev_angle[i], 0.528/2.0);
                // DEBUG: Print the LUT input and output to ensure functioning properly
                // std::cout << elev_angle[i] << " -> " << u << std::endl; 
                
                v = binarySearch(azim_LUT, azim_angle[i], 0.36/2.0);
                // DEBUG: Print the LUT input and output to ensure functioning properly
                // std::cout << azim_angle[i] << " -> " << v << std::endl;

                range_img.at<float>(u,v) = range[i]; // Physical range from 0 - 100 m (converting from mm --> m)
                index_img.at<float>(u,v) = i;
            }
            else
            {
                azim_angle[i]   = 0;
                elev_angle[i]   = 0;
            }

            // DEBUG: Print the computed range, azim angle, and elev angles
            // std::cout << "Point " << i << " / X " << pt.x << " / Y " << pt.y << " / Z " << pt.z << " / Range " << range[i] << " / Azim Angle " << azim_angle[i] <<  " / Elev Angle "  << elev_angle[i] << std::endl;
        }

        // Publish Range Image
        cv_bridge::CvImage range_msg;
        range_msg.encoding                  = sensor_msgs::image_encodings::TYPE_32FC1;
        range_msg.image                     = range_img;
        range_msg.header.stamp              = ros::Time::now();
        range_msg.header.frame_id           = cloud_in2->header.frame_id;        
        pub_range_img_.publish(range_msg.toImageMsg());
        
        return;
    }

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::getDepthImageOfficial(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_ros,
                                                    cv::Mat &range_img)
    {
        // start 0.038118956 seconds
        ouster_ros::OS1::CloudOS1 cloud_in{};
        pcl::fromROSMsg(*cloud_in_ros, cloud_in);
        
        // Get PCL metadata
        // for 64 channels, the px_offset =[ 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 
        //                                   0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18]

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
                // x_img.at<float>(u,v) = pt.x; // Physical x-coordinate (NORTH)
                // y_img.at<float>(u,v) = pt.y; // Physical y-coordinate (WEST)
                // z_img.at<float>(u,v) = pt.z; // Physical z-coordinate (UP)
                range_img.at<float>(u,v) = pt.range * 1e-3; // Physical range from 0 - 100 m (converting from mm --> m)
                // range_img.at<float>(u,v) = pt.range * 5e-3; // Range for 8-bit image from 0 - 255
                // noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);
                // intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);
            }
        }

        // Publish Range Image
        cv_bridge::CvImage range_msg;
        range_msg.encoding                      = sensor_msgs::image_encodings::TYPE_32FC1;
        range_msg.image                         = range_img;
        range_msg.header.stamp                  = ros::Time::now();
        range_msg.header.frame_id               = cloud_in_ros->header.frame_id;        
        pub_range_img_.publish(range_msg.toImageMsg());
    }

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::getPreFilterImage(cv::Mat &range_img, cv::Mat &filter_img)
    {
        cv::Mat return_img(H, W, CV_32FC1, 0.0);
        cv::Mat avg_range_img(H, W, CV_32FC1, 0.0);
        cv::Mat return_sum_img(H, W, CV_32FC1, 0.0);
        cv::Mat dev_range_img(H, W, CV_32FC1, 0.0);
        cv::Mat dev_range_bin_img(H, W, CV_32FC1, 0.0);
        cv::Mat filtered_img(H, W, CV_32FC1, 0.0);

        // Binarize depth (range) image
        cv::threshold(range_img, return_img, 0.10, 1.0, THRESH_BINARY);

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
        

        // DEBUNG: Print min and max, https://stackoverflow.com/questions/18233691/how-to-index-and-modify-an-opencv-matrix
        // double min = 0, max = 0;
        // cv::Point minLoc(-1, -1), maxLoc(-1, -1);
        
        // Image Filter

        // compute sum of positive matrix elements
        // (assuming that M isa double-precision matrix)
        double sum  = 0;
        int n_above = 2;
        int n_below = 2;
        int n_left  = 2;
        int n_right = 2;

        std::vector<float> kernel_vec = {1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                         1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                         1/sqrt(2), 1/sqrt(2),         0, 1/sqrt(2), 1/sqrt(2),
                                         1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2),
                                         1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2), 1/sqrt(2)};
        
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
                }
            }
        }

        cv::threshold(filtered_img, filter_img, fog_min_range_deviation_, 1.0, THRESH_TOZERO);
    }

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::labelForegroundFilterPCL(pcl::PointXYZI search_pt,
                                              pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2,
                                              pcl::PointIndices::Ptr inliers, 
                                              pcl::KdTreeFLANN<pcl::PointXYZI> kdtree,
                                              int i)
    {
        bool flag_fog;

        // If the point is within the intensity band of fog, then check out it's neighbors
        // if(search_pt.intensity > 0.04 && search_pt.intensity < 0.11)
        if(search_pt.intensity > fog_low_intensity_ && search_pt.intensity < fog_high_intensity_)
        {
            // Neighbors within radius search
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            
            // If it has neighbors
            if(kdtree.radiusSearch(search_pt, fog_radius_high_intensity_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                // Iterate over neighbors, and disregard if its neighbors have high-intensity points
                flag_fog = 1;
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    if(cloud_in2->points[pointIdxRadiusSearch[i]].intensity < fog_low_intensity_ ||
                       cloud_in2->points[pointIdxRadiusSearch[i]].intensity > fog_high_intensity_)
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

    // https://gist.github.com/mortenpi/f20a93c8ed3ee7785e65
    void FogDetectionNodelet::getPostFilterPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in2,
                                               pcl::PointIndices::Ptr inliers)
    {

        pcl::PointCloud<pcl::PointXYZI>::Ptr filt_fore(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filt_fore_z(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr fog(new pcl::PointCloud<pcl::PointXYZI>);

        if(inliers->indices.size() == 0)
        {
            // do nothing
        }
        else
        {

            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(cloud_in2);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*filt_fore);
            
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud (filt_fore);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits(-0.5, 2);
            pass.filter (*filt_fore_z);

            pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
            outrem.setInputCloud(filt_fore_z);
            outrem.setRadiusSearch(2.5);
            outrem.setMinNeighborsInRadius(4);
            outrem.setKeepOrganized(true);
            outrem.filter (*fog);

        }

            filt_fore->width = filt_fore->points.size();
            filt_fore->height = 1;
            filt_fore->is_dense = true;
            filt_fore->header.seq = cloud_in2->header.seq;
            filt_fore->header.frame_id = cloud_in2->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), filt_fore->header.stamp);
            pub_foreground_pcl_.publish (filt_fore);

            filt_fore_z->width = filt_fore_z->points.size();
            filt_fore_z->height = 1;
            filt_fore_z->is_dense = true;
            filt_fore_z->header.seq = cloud_in2->header.seq;
            filt_fore_z->header.frame_id = cloud_in2->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), filt_fore_z->header.stamp);
            pub_foreground_z_pcl_.publish (filt_fore_z);

            fog->width = fog->points.size();
            fog->height = 1;
            fog->is_dense = true;
            fog->header.seq = cloud_in2->header.seq;
            fog->header.frame_id = cloud_in2->header.frame_id;
            pcl_conversions::toPCL(ros::Time::now(), fog->header.stamp);
            pub_fog_pcl_.publish (fog);


    }

}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( fog::FogDetectionNodelet, nodelet::Nodelet)

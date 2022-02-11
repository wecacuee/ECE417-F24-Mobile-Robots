#include  <string>
#include  <iostream>


// sudo apt-get install libeigen3-dev
#include <Eigen/Dense>

// sudo apt-get install libopencv-dev
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


// sudo apt-get install ros-neotic-pcl-ros
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


#include "ros/ros.h"

cv::Mat
load_depth_image(const std::string& depth_img_filepath) {
    cv::Mat depthimg = cv::imread(depth_img_filepath, cv::IMREAD_ANYDEPTH);
    if (depthimg.data == NULL) {
        throw std::runtime_error("Unable to read " + depth_img_filepath);
    }
    double min_depth_i, max_depth_i;
    cv::minMaxIdx(depthimg, &min_depth_i, &max_depth_i);
    std::cout << "Min: " << min_depth_i << "; Max: " << max_depth_i << "\n";
    std::cout << "depthimg: CV_" << depthimg.type() << "C" << depthimg.channels() << "\n";

    cv::Mat depthimg_f;
    depthimg.convertTo(depthimg_f, CV_64FC1);
    assert(depthimg_f.channels() == 1);

    double max_depth, min_depth;
    cv::minMaxIdx(depthimg_f, &min_depth, &max_depth);
    std::cout << "Min: " << min_depth << "; Max: " << max_depth << "\n";
    return depthimg_f / 1000; // Convert to meters from millimeters
}


void
visualize_depth_image(const cv::Mat& depthimg_f) {
    double max_depth, min_depth;
    cv::minMaxIdx(depthimg_f, &min_depth, &max_depth);

    cv::imshow("Depth IMG", depthimg_f / max_depth); // Normalized image is good for visualization
    cv::waitKey(-1);
}

void
publish_point_cloud(const Eigen::MatrixXd& world_coords,
                    const ros::Publisher& pcd_publisher)  {
    // double max_z, min_z, max_x, min_x, max_y, min_y;
    // cv::minMaxIdx(world_coords.row(0), &min_x, &max_x);
    // std::cout << "x_min: " << min_x << "; x_max: " << max_x << "\n";
    // cv::minMaxIdx(world_coords.row(1), &min_y, &max_y);
    // std::cout << "y_min: " << min_y << "; y_max: " << max_y << "\n";
    // cv::minMaxIdx(world_coords.row(2), &min_z, &max_z);
    // std::cout << "z_min: " << min_z << "; z_max: " << max_z << "\n";

    PointCloud::Ptr msg(new PointCloud);
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    msg->header.frame_id = "map";
    msg->height = 1;
    msg->width = world_coords.cols();
    for (int i = 0; i < world_coords.cols(); ++i) {
        pcl::PointXYZ pt(world_coords(0, i),
                         world_coords(1, i),
                         world_coords(2, i));
        msg->points.push_back(pt);
    }



    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pcd_publisher.publish(msg);
}


int main(int argc, char**  argv)  {
    ros::init(argc, argv, "camera_model_example_node_with_eigen");

    ros::NodeHandle nh;
    // Declare a publisher with PointCloud type, on point_cloud topic with a queue size of 1
    // By using latch = true, we can publish once and it will be "latched". No need to publish at a high rate
    ros::Publisher pcd_publisher = nh.advertise<PointCloud>(/*topic=*/"point_cloud", /*queue_size=*/1, /*latch=*/true);

    // Gets the parameter  from the  Parameter server
    std::string depth_img_filepath;
    ros::param::get("~depth_img_filepath", depth_img_filepath);

    cv::Mat  depthimg_f  =   load_depth_image(depth_img_filepath);
    // visualize_depth_image(depthimg_f);
    Eigen::MatrixXd  depth_eig;
    cv::cv2eigen(depthimg_f,  depth_eig);

    //   Camera   intrinsic  matrix  3x3
    Eigen::Matrix3d K; K << 500,   0, 319.5,
                           0,   500, 239.5,
                           0,     0,    1;
    Eigen::Matrix3d  Kinv  =  K.inverse();
    Eigen::MatrixXd point_cloud_eigen(3,  depth_eig.rows()  * depth_eig.cols());
    for  (int  r =  0 ;  r  < depth_eig.rows() ; ++r)   {
        for (int   c   =  0;  c  < depth_eig.cols();  ++c)  {

            //  𝜆𝐮  = KX  
            //  X   =  ?
            //  X  =  𝜆K⁻¹𝐮
            //   𝐮  = [x, y,  1]  //  where  x  and  y are  image  coordinates
            //   𝐮 = [c + 0.5, r +  0.5,  1]
            Eigen::Vector3d  u;  u <<  c  +  0.5,  r  +  0.5,  1;
            //  X/𝜆  =  K⁻¹𝐮
            Eigen::Vector3d  X_by_lambda   =  Kinv   *   u;
            double  Z   =   depth_eig(r, c);
            Eigen::Vector3d  X =  X_by_lambda /  X_by_lambda(2)   *  Z;
            int idx  =  r*depth_eig.cols()  +   c;
            point_cloud_eigen.col(idx) =  X;
        }
    }

    publish_point_cloud(point_cloud_eigen,
                        pcd_publisher);

    ros::Rate loop_rate(10);
    while (nh.ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

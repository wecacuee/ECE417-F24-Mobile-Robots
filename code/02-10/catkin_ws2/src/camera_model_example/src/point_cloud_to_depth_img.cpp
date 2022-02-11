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

//  sudo  apt-get  install  ros-noetic-cv-bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>



// sudo apt-get install ros-noetic-pcl-ros
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


#include "ros/ros.h"

void
save_depth_image(const Eigen::MatrixXd depthimg_eig,
                 const std::string& depth_img_filepath) {
    cv::Mat  depthimg_f;
    cv::eigen2cv(depthimg_eig,  depthimg_f);

    depthimg_f *= 1000;  //  convert   meters  to millimeters
    cv::Mat  depthimg_16u;
    depthimg_f.convertTo(depthimg_16u, CV_16UC1);

    cv::imwrite(depth_img_filepath, depthimg_16u);
}

void
visualize_depth_image(const  Eigen::MatrixXd& depth_eig) {
    cv::Mat depthimg_f;
    cv::eigen2cv(depth_eig, depthimg_f);
    double max_depth, min_depth;
    cv::minMaxIdx(depthimg_f, &min_depth, &max_depth);

    cv::imshow("Depth IMG", depthimg_f / max_depth); // Normalized image is good for visualization
    cv::waitKey(-1);
}

Eigen::MatrixXd
point_cloud_to_eigen(const PointCloud::ConstPtr&  msg) {
    Eigen::MatrixXd world_coords(3, msg->points.size());
    for (int i = 0; i < msg->points.size(); ++i) {
        const  pcl::PointXYZ& pt  =  msg->points[i];
        world_coords(0,i) = pt.x;
        world_coords(1,i) = pt.y;
        world_coords(2,i) = pt.z;
    }
    return  world_coords;
}

void publish_depth_img(
    const ros::Publisher& depth_publisher,
    const  Eigen::MatrixXd&   depth_eig,
    const   pcl::PCLHeader& header)   {
    cv::Mat depthimg_cv;
    cv::eigen2cv(depth_eig, depthimg_cv);
    depthimg_cv *= 1000;  //  convert   meters  to millimeters
    cv::Mat  depthimg_16u;
    depthimg_cv.convertTo(depthimg_16u, CV_16UC1);
    std_msgs::Header  ros_header;
    pcl_conversions::fromPCL(header.stamp, ros_header.stamp);
    ros_header.frame_id = header.frame_id;
    cv_bridge::CvImage  cv_bridge_img(ros_header,  sensor_msgs::image_encodings::TYPE_16UC1,
                                      depthimg_16u);
    sensor_msgs::ImagePtr msg = cv_bridge_img.toImageMsg();
    depth_publisher.publish(msg);
}

class  PointCloudCallback {
public:
    PointCloudCallback(ros::NodeHandle&  nh)
        :
        nh_(nh),
        depth_publisher_(
            nh.advertise<sensor_msgs::Image>(/*topic=*/"depth_published", /*queue_size=*/1, /*latch=*/true))
        {
        }

    void callback(const PointCloud::ConstPtr  pcd_msg) {
      Eigen::MatrixXd  point_cloud_eigen  = point_cloud_to_eigen(pcd_msg);

      //   Camera   intrinsic  matrix  3x3
      Eigen::Matrix3d K; K << 500,   0, 319.5,
                            0,   500, 239.5,
                            0,     0,    1;
      Eigen::MatrixXd  depth_eig =  Eigen::MatrixXd::Zero(480,  640);  //  Initialize  everything  to  zero
      for  (int  idx  = 0; idx  < point_cloud_eigen.cols();  ++idx) {
          //  𝜆  𝐮  = ?
          Eigen::Vector3d lambda_u   =  Eigen::Vector3d::Ones(); // TODO:  Replace  this
          //
          double lambda  =  1; //  TODO: Replace this
          //  𝐮  = ?
          Eigen::Vector3d u  =  Eigen::Vector3d::Ones();;  //  TODO: Replace  this

          // Z  =  ?
          double  Z =  1; //  TODO:  Replace   this
          int  r  =  (int)round(u(1));    //  Compute   integer  row =  y  coordinate
          int  c   =  (int)round(u(0));   //  Compute   integer  col =  x  coordinate
          if ( 0 <= r &&  r  < depth_eig.rows()
               && 0 <=  c &&  c  < depth_eig.cols()  )
              {
              depth_eig(r,  c) =  Z;
          }
      }

      //  visualize_depth_image(depth_eig)
      publish_depth_img(depth_publisher_, depth_eig, pcd_msg->header);
  }

private:
    ros::NodeHandle& nh_;
    ros::Publisher depth_publisher_;
};

int main(int argc, char**  argv)  {
    ros::init(argc, argv, "point_cloud_to_depth_img");

    ros::NodeHandle nh;
    // Declare a publisher with PointCloud type, on point_cloud topic with a queue size of 1
    // By using latch = true, we can publish once and it will be "latched". No need to publish at a high rate
    PointCloudCallback pcd_callback(nh);
    ros::Subscriber pcd_subscriber = nh.subscribe(/*topic=*/"point_cloud", /*queue_size=*/1, /*callback=*/&PointCloudCallback::callback,   /*this=*/&pcd_callback);

    ros::Rate loop_rate(10);
    while (nh.ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

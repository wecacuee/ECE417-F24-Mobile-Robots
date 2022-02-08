#include <vector>

#include <ros/ros.h>

// sudo apt-get install libopencv-dev
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// sudo apt-get install ros-neotic-pcl-ros
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

cv::Mat
load_rgb_image(const std::string& rgb_img_filepath) {
    cv::Mat rgbimg = cv::imread(rgb_img_filepath, cv::IMREAD_ANYCOLOR);
    if (rgbimg.data == NULL) {
        throw std::runtime_error("Unable to read " + rgb_img_filepath);
    }
    return rgbimg;
}

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
visualize_rgb_image(const cv::Mat& rgbimg) {
    cv::imshow("RGB IMG", rgbimg);
    cv::waitKey(-1);
}

/**
 * Generates coordinates for every pixel
 */
cv::Mat_<double>
create_hom_pixel_coordinates(const cv::MatSize& shape) {
    int npixels = shape[0]*shape[1];
    cv::Mat_<double> hom_pixel_coords(3, npixels);
    for (int v = 0; v < shape[0]; ++v) {
        for (int u = 0; u < shape[1]; ++u) {
            hom_pixel_coords(0, v*shape[1] + u) = u + 0.5;
            hom_pixel_coords(1, v*shape[1] + u) = v + 0.5;
            hom_pixel_coords(2, v*shape[1] + u) = 1;
        }
    }
    return hom_pixel_coords;
}

void
publish_point_cloud(const cv::Mat_<double>& world_coords,
                    const ros::Publisher& pcd_publisher,
                    const cv::Mat rgbimg) {
    double max_z, min_z, max_x, min_x, max_y, min_y;
    cv::minMaxIdx(world_coords.row(0), &min_x, &max_x);
    std::cout << "x_min: " << min_x << "; x_max: " << max_x << "\n";
    cv::minMaxIdx(world_coords.row(1), &min_y, &max_y);
    std::cout << "y_min: " << min_y << "; y_max: " << max_y << "\n";
    cv::minMaxIdx(world_coords.row(2), &min_z, &max_z);
    std::cout << "z_min: " << min_z << "; z_max: " << max_z << "\n";

    PointCloud::Ptr msg(new PointCloud);
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    msg->header.frame_id = "map";
    msg->height = rgbimg.rows;
    msg->width = rgbimg.cols;
    for (int i = 0; i < world_coords.cols; ++i) {
        int r = i / rgbimg.cols;
        int c = i % rgbimg.cols;
        cv::Vec3b bgr = rgbimg.at<cv::Vec3b>(r, c);
        pcl::PointXYZRGB pt(bgr(2), bgr(1), bgr(0));
        pt.x = world_coords(0, i);
        pt.y = world_coords(1, i);
        pt.z = world_coords(2, i);
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


int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_model_example_node");
    std::string depth_img_filepath;
    if ( ! ros::param::get("~depth_img_filepath", depth_img_filepath)) {
        throw std::runtime_error("Private parameter ~depth_img_filepath is required");
    }
    std::string rgb_img_filepath;
    if ( ! ros::param::get("~rgb_img_filepath", rgb_img_filepath)) {
        throw std::runtime_error("Private parameter ~rgb_img_filepath is required");
    }

    cv::Mat depthimg_f = load_depth_image(depth_img_filepath);
    // visualize_depth_image(depthimg_f);

    cv::Mat rgbimg = load_rgb_image(rgb_img_filepath);
    cv::Mat rgbimg_depthsize;
    cv::resize(rgbimg, rgbimg_depthsize, depthimg_f.size());
    // visualize_rgb_image(rgbimg_f);

    ros::NodeHandle nh;
    // Declare a publisher with PointCloud type, on point_cloud topic with a queue size of 1
    // By using latch = true, we can publish once and it will be "latched". No need to publish at a high rate
    ros::Publisher pcd_publisher = nh.advertise<PointCloud>(/*topic=*/"point_cloud", /*queue_size=*/1, /*launch=*/true);

    /*****************************************************************************************************
     * 7th Feb Lecture's math begins here
     *****************************************************************************************************/
    cv::Matx33d K(700,   0, 319.5,
                  0,   700, 239.5,
                  0,     0,    1);
    // Kinv = K⁻¹
    cv::Matx33d Kinv = K.inv();

    // hom_pixel_coords = 𝐮
    cv::Mat_<double> hom_pixel_coords = create_hom_pixel_coordinates(depthimg_f.size); // 3 x n matrix where n = rows x cols

    // unscaled_world_coords = K⁻¹𝐮
    cv::Mat_<double> unscaled_world_coords = Kinv * hom_pixel_coords;
    cv::Mat_<double> world_coords(unscaled_world_coords.rows, unscaled_world_coords.cols);

    // world_coords = 𝐗 = 𝜆K⁻¹𝐮
    // 𝐗 = [X, Y, Z]
    // world_coords.row(2) = Z = depth[u, v] for all u, v
    world_coords.row(2) = 1*depthimg_f.reshape(1, 1); // Convert 480x640 depth image into a 1xn array.

    // 𝜆 = Z/[K⁻¹𝐮]₃   // where Z = depth
    cv::Mat_<double> lambda = depthimg_f.reshape(1, 1) / unscaled_world_coords.row(2);

    // world_coords.rows(0) = X = 𝜆K⁻¹𝐮
    world_coords.row(0) = lambda.mul(unscaled_world_coords.row(0));
    // world_coords.rows(1) = Y = 𝜆K⁻¹𝐮
    world_coords.row(1) = lambda.mul(unscaled_world_coords.row(1));

    /*****************************************************************************************************
     * 7th Feb Lecture's math ends here
     *****************************************************************************************************/


    publish_point_cloud(world_coords, pcd_publisher, rgbimg_depthsize);
    ros::Rate loop_rate(10);
    while (nh.ok()) {

        ros::spinOnce();

        loop_rate.sleep();
    }



    return 0;
}

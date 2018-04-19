#include <string>

// opencv3
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>


#include "rgbd2vr.hpp"


using namespace std;
using namespace cv;

RGBD2VR *vrRenderer = nullptr;

void process(const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::ImageConstPtr& depth_msg) {

    if (vrRenderer == nullptr) return;

    auto rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image; 
    auto depth = cv_bridge::toCvShare(depth_msg)->image; 

    vrRenderer->setNextBackgroundFrames(rgb, depth);

    //uint16_t *image_data = (uint16_t *) depth.data;

    //float g_depth_avg;
    //double depth_total = 0;
    //int depth_count = 0;
    //for (unsigned int i = 0; i < depth_msg->height * depth_msg->width; ++i)
    //{
    //    //if ((0 < *image_data) && (*image_data <= g_max_z))
    //    if ((0 < *image_data))
    //    {
    //        depth_total += *image_data;
    //        depth_count++;
    //    }
    //    image_data++;
    //}
    //if (depth_count != 0)
    //{
    //    g_depth_avg = static_cast<float>(depth_total / depth_count);
    //}

    //ROS_INFO_STREAM("Avg depth: " << g_depth_avg);

    //depth.convertTo(depth, CV_32F); // thresholding works on CV_8U or CV_32F but not CV_16U
    //imshow("Input depth", depth);
    //threshold(depth, depth, 0.5, 3.0, THRESH_BINARY_INV);
    //depth.convertTo(depth, CV_8U); // masking requires CV_8U. All non-zero values are kept, so '1.0' is fine

    //Mat maskedImage;
    //rgb.copyTo(maskedImage, depth > 0);

    //imshow("Input RGB", rgb);
    //imshow("Masked input", maskedImage);
    //waitKey(10);
}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "rgbd2vr");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(rosNode, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(rosNode, "depth", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_sub, depth_sub, 3);
    sync.registerCallback(bind(&process, _1, _2 ) );

    vrRenderer = new RGBD2VR();

    if (!vrRenderer->BInit())
    {
        vrRenderer->shutdown();
        return 1;
    }

    ROS_INFO("VR rendered successfully initialized! Starting to stream...");
    ros::Rate r(30); // Hz

    vrRenderer->start();

    while (!vrRenderer->shutdownRequested)
    {
        vrRenderer->step();
        ros::spinOnce();
        r.sleep();
    }

    vrRenderer->stop();

    vrRenderer->shutdown();


    ROS_INFO("rgbd2vr is ready. Waiting for pair of {rgb, depth} images!");
    ros::spin();

    return 0;
}


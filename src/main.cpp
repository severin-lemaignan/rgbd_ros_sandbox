#include <string>

// opencv3
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;

void process(const sensor_msgs::ImageConstPtr& msg,
             const sensor_msgs::CameraInfoConstPtr& camerainfo) {

    // inputImage is our OpenCV image. Attention! BGR, not RGB!
    auto inputImage = cv_bridge::toCvShare(msg)->image; 
    // convert to RGB, but this incurs a copy!
    //auto inputImage = cv_bridge::toCvShare(msg, "bgr8")->image; 

    ROS_INFO("Got an image!");

    imshow("Input", inputImage);
    waitKey(10);
}

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "ros_opencv_sandbox");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");

    image_transport::ImageTransport it(rosNode);
    auto sub = it.subscribeCamera("image", 1, &process);

    ROS_INFO("ros_opencv_sandbox is ready. Waiting for images!");
    ros::spin();

    return 0;
}


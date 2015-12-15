/*

	HUMANOID ROBOTIC SYSTEMS
	Final Project

	Vision Part

	Group B

	Adnan Makhani
	Andreas Plieninger
	Sebastian Wagner
	Tim Bruedigam

*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/TactileTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_listener.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedActionGoal.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_broadcaster.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"


// Include customn messages:
#include "RoNAOldo/visionMsg.h"
#include "RoNAOldo/controlMsg.h"

//using namespace aruco;
using namespace cv;
using namespace std;
using namespace ros;

// Thread for spinning:bool stop_thread=false;
bool stop_thread=false;

void spinThread()
{
    while(!stop_thread)
    {
        ros::spinOnce();
        //ROS_INFO_STREAM("Spinning the thing!!");
    }
}

// Vision Class
class Vision
{
public:

    // ros handler
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber visionSub;

    // publisher
    ros::Publisher visionPub;

    //Image tope camera subscriber for ball
    image_transport::Subscriber top_sub;


    // Spin Thread:
    boost::thread *spin_thread;

    //Message container to be sent to the control part
    RoNAOldo::visionMsg v_msg;

    // Image transporter
    image_transport::ImageTransport it;

    //Camera parameters for aruco marker detection
    aruco::CameraParameters cameraParameters;

    // Initialize Marker Class:
   aruco::MarkerDetector MDetector;
   vector<aruco::Marker> Markers;

    // Images and other variables required for template matching:
    Mat image;
    Mat image_goal;
    Mat image_hsv;
    Mat image_undist;
    Mat image_ball;
    Mat image_template;
    Mat image_ROI;
    Mat image_ROI_hsv;
    Rect region_of_interest;
    vector<cv::Mat> v_channel, image_channel;
    Mat mask, image_mask;
    Mat dist, camMatrix;
    MatND hist, backproj;
    float k1, k2, k3, k4, k5;
    float f1, f2, alpha, c1, c2;
    float hranges[2]= { 0, 180 };
    const float* ranges[1] = { hranges };
    int hsize[1] = { 180 };
    int chnls[1] = {0};

    bool ball_detected;



    // Counter: <just for testing>
    int count;

    Vision(NodeHandle n): it(nh_) {

  		nh_ = n;

  		visionSub = nh_.subscribe("controlMessage", 10, &Vision::controlMessageCallback, this);

  		visionPub = nh_.advertise<RoNAOldo::visionMsg>("visionMessage", 10);

    }

    ~Vision() {

      stop_thread=true;
      sleep(1);
      spin_thread->join();

    }

    void main_loop() {

      //int count = 0;
      ros::Rate rate_sleep(10);
      while(nh_.ok())
      {

        // Create Dummy Message:
        RoNAOldo::visionMsg msg;
        cout << "\nEnter Distance to Ball [m]: ";
        cin >> msg.ball_distance;
        cout << "\nEnter ball position relative to goal: ";
        cin >> msg.ball_off_middle;
        cout << "\nEnter ball position relative to image: ";
        cin >> msg.ball_rel_image;

        // Send Dummy Message:
        visionPub.publish(msg);

        rate_sleep.sleep();

      }

    }

    void controlMessageCallback(const RoNAOldo::controlMsg::ConstPtr &inMessage) {

        cout << inMessage->targetReached << endl;

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RoNAOldo_dummyVision");

    ros::NodeHandle n;


    startWindowThread();

    Vision vNode(n);
    vNode.main_loop();

    cout << "Vision has terminated!" << endl;

    return 0;

}

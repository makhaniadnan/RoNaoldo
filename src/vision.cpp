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
    image_transport::Subscriber ball_top_sub;

    // Spin Thread:
    boost::thread *spin_thread;

    //Message container to be sent to the control part
    RoNAOldo::visionMsg msg;

    // Image transporter
    image_transport::ImageTransport it;

    // Images and other variables required for template matching:
    Mat image;
    Mat image_hsv;
    Mat image_template;
    Mat image_ROI;
    Mat image_ROI_hsv;
    Rect region_of_interest;
    vector<cv::Mat> v_channel, image_channel;
    Mat mask, image_mask;
    MatND hist, backproj;
    float hranges[2]= { 0, 180 };
    const float* ranges[1] = { hranges };
    int hsize[1] = { 180 };
    int chnls[1] = {0};



    // Counter: <just for testing>
    int count;

    Vision(NodeHandle n): it(nh_) {

		nh_ = n;

		visionSub = nh_.subscribe("controlMessage", 10, &Vision::controlMessageCallback, this);

		visionPub = nh_.advertise<RoNAOldo::visionMsg>("visionMessage", 10);

		count = 0;
    //Ball tracking initializations



    // read image and extract area of interest from template img
    try
    {
        //image_template = imread("/home/hrs2015/catkin_ws/src/RoNaoldo/images/mor_temp.jpg");
        image_template = imread("/home/hrs2015/catkin_ws/src/RoNaoldo/images/nit_temp.jpg");


        if( image_template.empty() )  // Check for invalid input
        {
            cout << "Could not open or find the image" << endl ;
        }
        //region_of_interest = Rect(250,253,189,189);  // for morning
        region_of_interest = Rect(269,269,163,163);  // for night

        image_ROI = image_template(region_of_interest);
        imshow("ROI", image_ROI);
        waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Couldn't extract ROI");
    }
    // Convert to HSV ROI :
    try {
        cvtColor(image_ROI,image_ROI_hsv,CV_BGR2HSV);
        split(image_ROI_hsv,v_channel);
        threshold(v_channel[1], mask, 70, 255, THRESH_BINARY);
        calcHist(&image_ROI_hsv,1,chnls,mask,hist, 1,hsize, ranges); //calculate histogram
	      normalize(hist, hist, 0, 255, NORM_MINMAX, -1, Mat() ); // normalize histogram

        //below commented code is for printing histogram
	      int rows = 64;
        //default height size
        int cols = hist.rows;
        //get the width size from the histogram
        float scaleX = 3;
        float scaleY = 3;
        Mat histImg = Mat::zeros(rows*scaleX, cols*scaleY, CV_8UC3);

        for(int i=0; i<cols-1; i++) {
            float histValue = hist.at<float>(i,0);
            float nextValue = hist.at<float>(i+1,0);
            Point pt1 = Point(i*scaleX, rows*scaleY);
            Point pt2 = Point(i*scaleX+scaleX, rows*scaleY);
            Point pt3 = Point(i*scaleX+scaleX, (rows-nextValue*rows/255)*scaleY);
            Point pt4 = Point(i*scaleX, (rows-nextValue*rows/255)*scaleY);
            int numPts = 5;
            Point pts[] = {pt1, pt2, pt3, pt4, pt1};
            fillConvexPoly(histImg, pts, numPts, Scalar(255,255,255));
        }
        imshow("hist", histImg);
    }
    catch (...) {
        ROS_ERROR("Error in computing histogram!");
    }


    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);

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
        msg.ballArea = count;
        count++;
        visionPub.publish(msg);
        ball_top_sub = it.subscribe("nao/nao_robot/camera/top/camera/image_raw", 1, &Vision::detect_Ball, this);
        rate_sleep.sleep();
      }

    }
    void detect_Ball(const sensor_msgs::ImageConstPtr& msg)
    {



      // Convert to bgr8 and display:
      try
      {
          //bgr8: CV_8UC3, color image with blue-green-red color order
          image = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
      // Back Projection + mean/cam shift tracking:
      try {
          cvtColor(image,image_hsv,CV_BGR2HSV);
          split(image_hsv,image_channel);
          threshold(image_channel[1], image_mask, 70, 255, THRESH_BINARY);
          calcBackProject( &image_channel[0], 1, chnls, hist, backproj, ranges); //calculate backprojection
          bitwise_and(backproj,image_mask,backproj);
          //meanShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1));
          CamShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1));
          rectangle(image, region_of_interest,1,2,1,0);
          imshow("Tracking", image);
          waitKey(30);
      }
      catch (...) {
          ROS_ERROR("Error in meanshift/ camshift!");
      }


    }


    void controlMessageCallback(const RoNAOldo::controlMsg::ConstPtr &inMessage) {

        cout << inMessage->targetReached << endl;

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RoNAOldo_vision");

    ros::NodeHandle n;
    namedWindow("ROI");
    namedWindow("hist");
    namedWindow("Tracking");

    startWindowThread();

    Vision vNode(n);
    vNode.main_loop();

    cout << "Vision has terminated!" << endl;

    return 0;

}

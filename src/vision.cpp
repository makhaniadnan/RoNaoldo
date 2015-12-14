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



    // Counter: <just for testing>
    int count;

    Vision(NodeHandle n): it(nh_) {

		nh_ = n;

		visionSub = nh_.subscribe("controlMessage", 10, &Vision::controlMessageCallback, this);

		visionPub = nh_.advertise<RoNAOldo::visionMsg>("visionMessage", 10);

		count = 0;
    // Define Camera Parameters:
    k1 = -0.066494;
    k2 = 0.095481;
    k3 = -0.000279;
    k4 = 0.002292;
    k5 = 0.000000;
    f1 = 551.543059;
    f2 = 553.736023;
    alpha = 0.0;
    c1 = 327.382898;
    c2 = 225.026380;
    float alphaf1 = alpha * f1;

    // Init Camera Parameters:
    dist = (Mat_<float>(1,5) << k1, k2, k3, k4, k5);
    camMatrix = (Mat_<float>(3,3) << f1, alphaf1, c1, 0.0, f2, c2, 0.0, 0.0, 1.0);

    cameraParameters.setParams(camMatrix, dist, cv::Size(640,480));
	  cameraParameters.resize(cv::Size(640,480));

    // read image and extract area of interest from template img
    try
    {
        //image_template = imread("/home/hrs2015/catkin_ws/src/RoNaoldo/images/mor_temp.jpg");
        image_template = imread("/home/hrs2015/catkin_ws/src/RoNaoldo/images/nit_temp.jpg");


        if( image_template.empty() )  // Check for invalid input
        {
            cout << "Could not open or find the image" << endl ;
        }
        //region_of_interest = Rect(250,253,189,189);  // for mor_temp
        region_of_interest = Rect(269,269,163,163);  // for nit_temp

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
        v_msg.ballArea = count;
        count++;
        visionPub.publish(v_msg);
        top_sub = it.subscribe("nao/nao_robot/camera/top/camera/image_raw", 1, &Vision::detect_Ball_Goal, this);

        rate_sleep.sleep();
      }

    }
    void detect_Ball_Goal(const sensor_msgs::ImageConstPtr& msg)
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
          image_ball = image.clone();
          image_goal = image.clone();
          cvtColor(image,image_hsv,CV_BGR2HSV);
          split(image_hsv,image_channel);
          threshold(image_channel[1], image_mask, 70, 255, THRESH_BINARY);
          calcBackProject( &image_channel[0], 1, chnls, hist, backproj, ranges); //calculate backprojection
          bitwise_and(backproj,image_mask,backproj);
          //meanShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1));
          CamShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1));
          rectangle(image_ball, region_of_interest,1,2,1,0);
          imshow("Tracking", image_ball);
          waitKey(30);
      }
      catch (...) {
          ROS_ERROR("Error in meanshift/ camshift!");
      }

      // Calculate distance to the ball
      try {

        undistort(image,image_undist,camMatrix,dist);
        float distance = f2 * (100.00/ (float) region_of_interest.height);
        cout << "dist"<< distance << endl<<endl;




      }
      catch (...) {
          ROS_ERROR("cant get the distance to the ball");
      }

     // Detect Markers
      try {

        // Initialize Marker Class:
       aruco::MarkerDetector MDetector;
       vector<aruco::Marker> Markers;

      // Detect Marker:
      // size of marker
      float markerSize = 0.135; //13.5mm
      MDetector.detect(image, Markers, cameraParameters, markerSize);

      // Draw  Marker Info:
      double pos3d[3];
      double orient3d[4];
      for (unsigned int i=0;i<Markers.size();i++) {
      //cout<<Markers[i]<<endl;
      //ROS_INFO("Found Marker %d", Markers[i].id);
        Markers[i].draw(image_goal, cv::Scalar(0,0,255),2);

       // Get 3D position and print it:
        Markers[i].OgreGetPoseParameters(pos3d, orient3d);
       //  ROS_INFO("Marker's 3D position: X: %f - Y: %f - Z: %f", pos3d[0], pos3d[1], pos3d[2]);
        cv::Point2f markerCenter;
        markerCenter = Markers[i].getCenter();

        //ROS_INFO("Marker's center is X: %f - Y: %f",markerCenter.x,markerCenter.y);

        }
       // Show image:
         imshow("goal", image_goal);
           //wait 30ms
         cv::waitKey(30);



      }
      catch (...) {
          ROS_ERROR("Cant Detect Markers");
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
    namedWindow("goal");


    startWindowThread();

    Vision vNode(n);
    vNode.main_loop();

    cout << "Vision has terminated!" << endl;

    return 0;

}

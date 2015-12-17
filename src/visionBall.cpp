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
#include "RoNAOldo/ballMsg.h"

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
    ros::Publisher ballCenterPub;

    //Image tope camera subscriber for ball
    image_transport::Subscriber top_sub;


    // Spin Thread:
    boost::thread *spin_thread;

    //Message container to be sent to the control part
    RoNAOldo::visionMsg v_msg;

    // Image transporter
    image_transport::ImageTransport it;

    // Images and other variables required for template matching:
    Mat image;
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

    Rect color_camshift_region_of_interest = Rect(229,325,92,92);  // for mor_temp


    // Counter: <just for testing>
    int count;

    Vision(NodeHandle n): it(nh_) {

		nh_ = n;

    ballCenterPub = nh_.advertise<RoNAOldo::ballMsg>("ball", 10);

		count = 0;

    ball_detected = false;
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

        top_sub = it.subscribe("image", 1, &Vision::detect_Ball, this);

        rate_sleep.sleep();
      }

    }


    void detectBallUsingColorBlob(Mat image)
    {
      Mat image_color;
      Mat image_hsv;
      Mat image_dilate;
      // Convert to HSV and extract colors:
      try {
        cvtColor(image,image_hsv,CV_BGR2HSV);

        Scalar redMin = Scalar(0*180,0.4*255,0.8*255);
        Scalar redMax = Scalar(30.0/360.0*180,1*255,1*255);

        inRange(image_hsv, redMin, redMax, image_color);
        imshow("Colour extraction", image_color);


        //imshow("Blob Extraction", image_dilate);
        waitKey(30);
      }
      catch (...) {
        ROS_ERROR("Error in Colour Extraction!");
      }

      // Blob Extraction:
      Mat image_keypoints;
      vector<KeyPoint> keypoints;
      try{
        Scalar redMin2 = Scalar(0*180,0.2*255,0.8*255);
        Scalar redMax2 = Scalar(30.0/360.0*180,1*255,1*255);

        Mat image_hsv_blob;
        inRange(image_hsv, redMin2, redMax2, image_hsv_blob);
        imshow("HSV blob extraction", image_hsv_blob);

        // Dilate and/or Erode:
        int erosion_size = 5;
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
        dilate(image_hsv_blob, image_dilate, element);
        erode( image_dilate, image_dilate, element );

        // Initialize Blob detector class:
        SimpleBlobDetector::Params params;
        params.minDistBetweenBlobs = 50.0f;
        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;
        params.filterByCircularity = true;
        params.filterByArea = true;
        params.minArea = 400.0f;
        params.maxArea = 3000.0f;
        params.minCircularity = 0.4f;
        params.maxCircularity = 1.0f;
        params.minConvexity = 0.5;
        params.maxConvexity = 1.0;
        params.minInertiaRatio = 0.4;
        params.maxInertiaRatio = 1.0;
        SimpleBlobDetector blob_detector(params);

        // detect the blobs keypoints (center of mass):
        blob_detector.detect(image_dilate, keypoints);

        drawKeypoints(image_dilate, keypoints, image_keypoints);

        // extract the x y coordinates of the keypoints:
        for (int i=0; i<keypoints.size(); i++){
           float X = keypoints[i].pt.x;
           float Y = keypoints[i].pt.y;
           ROS_INFO("DETECTED BLOB %f %f", X, Y );

          //draw keypoiints
           circle(image_keypoints, Point(X,Y), 30, Scalar(255,0,0), 10);
        }


      }
      catch (...) {
        ROS_ERROR("Error in Blob Detection!");
      }


      //ensure it is in valid area&
      color_camshift_region_of_interest = color_camshift_region_of_interest & Rect(0, 0, image_color.cols, image_color.rows);
      if(color_camshift_region_of_interest.area() < 1) {
          color_camshift_region_of_interest = Rect(0, 0, image_color.cols, image_color.rows);
      }
      cout << "region of interest" << color_camshift_region_of_interest << endl;

      //termcriteria is type, maxCount, epsilon (changes smaller than epsilon, stop searching)
      CamShift(image_color,color_camshift_region_of_interest, TermCriteria((TermCriteria::COUNT || TermCriteria::EPS), 40, 1));

      cout << "region of interest" << color_camshift_region_of_interest << endl;

      const Scalar rect_color = Scalar(0,255,0);
      rectangle(image_keypoints, color_camshift_region_of_interest,rect_color,5);

      ball_detected = false;
      double rect_center_x = 0;
      double rect_center_y = 0;
      double ball_distance = 0;
      //focal length of top camera
      double f2 = 553.736023;
      //if camshift detected a "ball"

      if(color_camshift_region_of_interest.area() > 1.0) {
        //check if blob detection also got one
        rect_center_x = color_camshift_region_of_interest.x + (color_camshift_region_of_interest.width/2.0);
        rect_center_y = color_camshift_region_of_interest.y + (color_camshift_region_of_interest.height/2.0);
        ball_distance = f2 * (70.00/ (0.5*(color_camshift_region_of_interest.height + color_camshift_region_of_interest.width)));


        // extract the x y coordinates of the keypoints:
        for (int i=0; i<keypoints.size(); i++){
           double distance = norm(keypoints[i].pt - Point2f(rect_center_x,rect_center_y));
           ROS_INFO("distance is %f", distance);
           if(distance < 10.0) {
             ROS_INFO("found our ball in camshift and blob detection");
             ball_detected = true;
             break;
           }
        }
      }


      //draw keypoints
      //drawKeypoints(image_dilate, keypoints, image_keypoints);
      imshow("Blob Extraction", image_keypoints);


      try {
        if(ball_detected) {
          RoNAOldo::ballMsg ballMsg;

          ballMsg.ball_distance = ball_distance / 1000.0;
          ballMsg.ball_center_x = (float) rect_center_x;
          ballMsg.image_width = image.cols;

          ballCenterPub.publish(ballMsg);
        }
      } catch (...) {
          ROS_ERROR("cannot publish ball position");
      }


      //result: HoughCircles dont work
      /*
      //try to use houghcircles here
      vector<Vec3f> circles;

      /// Apply the Hough Transform to find the circles
      HoughCircles( image_color, circles, CV_HOUGH_GRADIENT, 1, image_color.rows/8, 200, 100, 0, 0 );

      /// Draw the circles detected
      for( size_t i = 0; i < circles.size(); i++ )
      {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( image_keypoints, center, 3, Scalar(255,0,0), -1, 8, 0 );
          // circle outline
          circle( image_keypoints, center, radius, Scalar(255,0,0), 3, 8, 0 );
          ROS_INFO("deteced houghcircle");
       }
       */


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

      //try to detect ball using blop and color Detection
      detectBallUsingColorBlob(image);



      // Back Projection + mean/cam shift tracking:
      try {
          ball_detected = false;
          image_ball = image.clone();
          cvtColor(image,image_hsv,CV_BGR2HSV);
          split(image_hsv,image_channel);
          threshold(image_channel[1], image_mask, 70, 255, THRESH_BINARY);
          calcBackProject( &image_channel[0], 1, chnls, hist, backproj, ranges); //calculate backprojection
          bitwise_and(backproj,image_mask,backproj);
          //meanShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 10, 1));
          //if( region_of_interest.height < 0.0 || region_of_interest.width < 0.0 || region_of_interest.x || region_of_interest.y)
          region_of_interest = Rect(229,325,92,92);  // for mor_temp
                CamShift(backproj,region_of_interest, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 20, 1));
                rectangle(image_ball, region_of_interest,1,2,1,0);
                if( ( (float)region_of_interest.height/(float)region_of_interest.width) < 1.5 && ((float)region_of_interest.width/(float)region_of_interest.height ) > .6 )
                { ball_detected = true; }



          imshow("Tracking", image_ball);
          imshow("Backproj",backproj);
          waitKey(30);
      }
      catch (...) {
          ROS_ERROR("Error in meanshift/ camshift!");
      }

      // Prepare message for control

      //define it, so wen can always use it for publish
      float distance = -1.0;
      try {
        float ball_pos_goal = -1.0;
        float ball_pos_img = -1.0;
        undistort(image,image_undist,camMatrix,dist);
        if ( region_of_interest.height >0)
        {
          distance = f2 * (70.00/ (float) region_of_interest.height);
        }
      } catch (...) {
          ROS_ERROR("cant get the distance to the ball");
      }

      /*
      publishh it from other functio now
      try {
        if(ball_detected) {
          RoNAOldo::ballMsg ballMsg;

          ballMsg.ball_distance = distance;
          ballMsg.ball_center_x = region_of_interest.x + (region_of_interest.width/2);

          ballCenterPub.publish(ballMsg);
        }
      } catch (...) {
          ROS_ERROR("cannot publish ball position");
      }*/

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "RoNAOldo_vision");

    ros::NodeHandle n;
    namedWindow("ROI");
    namedWindow("hist");
    namedWindow("Tracking");
    namedWindow("Backproj");
    namedWindow("goal");
    namedWindow("Colour extraction");
    namedWindow("Blob Extraction");
    namedWindow("HSV blob extraction");



    startWindowThread();

    Vision vNode(n);
    vNode.main_loop();

    cout << "Vision has terminated!" << endl;

    return 0;

}

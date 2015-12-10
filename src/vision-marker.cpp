#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

//include custom messages
#include "RoNAOldo/controlMsg.h"

namespace vision {

	aruco::CameraParameters cameraParameters;

	cv::Point2f lastBallCenter;

	void initCameraParameters(void)
	{
	  // Define Camera Parameters:
	  float k1 = -0.066494;
	  float k2 = 0.095481;
	  float k3 = -0.000279;
	  float k4 = 0.002292;
	  float k5 = 0.000000;
	  float f1 = 551.543059;
	  float f2 = 553.736023;
	  float alpha = 0.0;
	  float c1 = 327.382898;
	  float c2 = 225.026380;
	  float zero = 0.0;
	  float one = 1.0;
	  float alphaf1 = alpha * f1;

	  // Init Camera Parameters:
	  cv::Mat dist = (cv::Mat_<float>(1,5) << k1, k2, k3, k4, k5);
	  cv::Mat camMatrix = (cv::Mat_<float>(3,3) << f1, alphaf1, c1, zero, f2, c2, zero, zero, one);
	  cameraParameters.setParams(camMatrix, dist, cv::Size(640,480));
	  cameraParameters.resize(cv::Size(640,480));

	  //TODO get from callback
	  lastBallCenter.x = 233.0;
	  lastBallCenter.y = 5.0;
	}

	void ballPositionCallback(const RoNAOldo::controlMsg::ConstPtr &inMessage)
	{
		ROS_INFO("updatated ball position");
		cout << inMessage->targetReached << endl;

		lastBallCenter.x = inMessage->ballPositionX;
		lastBallCenter.y = inMessage->ballPositionY;
		ROS_INFO("ball position now is X: %f - Y: %f", lastBallCenter.x, lastBallCenter.y);
	}

	void markerImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{

		ROS_INFO("got image callback");	


	  // Images:
	  cv::Mat image;

	  // Convert to bgr8 and display:
	  try
	  {
	    image = cv_bridge::toCvShare(msg, "bgr8")->image;
	    //cv::imshow("Original Image", image);
	    //show image 30ms
	    //cv::waitKey(30);
	  }
	  catch (cv_bridge::Exception& e) {
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }

	  // 3D Marker Detection:
	  try {
	    // Initialize Marker Class:
	    aruco::MarkerDetector MDetector;
	    vector<aruco::Marker> Markers;

	    // Detect Marker:
		// size of marker
		float markerSize = 0.135; //13.5mm
	    MDetector.detect(image, Markers, cameraParameters, markerSize);

	    // Draw  Marker Info:
	    cv::Mat image_marker = image;
	    double pos3d[3];
	    double orient3d[4];
	    for (unsigned int i=0;i<Markers.size();i++) {
	      //cout<<Markers[i]<<endl;
	      ROS_INFO("Found Marker %d", Markers[i].id);

	      Markers[i].draw(image_marker, cv::Scalar(0,0,255),2);
	    
	      // Get 3D position and print it:
	      Markers[i].OgreGetPoseParameters(pos3d, orient3d);
	      ROS_INFO("Marker's 3D position: X: %f - Y: %f - Z: %f", pos3d[0], pos3d[1], pos3d[2]);
		  
		  cv::Point2f markerCenter;
		  markerCenter = Markers[i].getCenter();

		  ROS_INFO("Marker's center is X: %f - Y: %f",markerCenter.x,markerCenter.y);

	    }

		//coordinate system of opencv is as follows:
		//top left corner here
		//(tl) --- x --->
		// |
		// |
		// y
		// |
		// \/

		//draw ball marker
		cv::circle(image_marker, lastBallCenter, 30, cv::Scalar(0,255,0), 3);



		//check if we found both markers
		if(Markers.size() == 2) {
			//calculate middle of the two markers

			float horizontalMiddle = Markers[0].getCenter().x + 0.5 * (Markers[1].getCenter().x - \
					Markers[0].getCenter().x);

			ROS_INFO("Middle (x) of two markers is %f", horizontalMiddle);

			//draw middle line
			cv::line(image_marker, cv::Point2f(horizontalMiddle,0.0), cv::Point2f(horizontalMiddle,480.0), cv::Scalar(255,0,0));

			//is ball left or right from middle?

			float ballOffMiddle = lastBallCenter.x - horizontalMiddle;

			if(ballOffMiddle >= 0) {
				ROS_INFO("Ball is RIGHT of middle (or in the middle): %f pixels", ballOffMiddle);
			} else {
				ROS_INFO("Ball is LEFT of middle: %f pixels", -ballOffMiddle);
			}
		}

	    // Show image:
		cv::imshow("3D marker position", image_marker);
		//wait 30ms
	    cv::waitKey(30);

		//if at least one marker was found
		if(Markers.size() > 0) {
			// wait until key pressed
			cv::waitKey(0);
		}
	  }
	  catch (...) {
	    ROS_ERROR("Error in Marker Detection!");
	  }

	}

} //namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_marker");
  ros::NodeHandle nh;

  vision::initCameraParameters();

  cv::namedWindow("3D marker position");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, vision::markerImageCallback);

  //subscribe to ball position
  ros::Subscriber sub2 = nh.subscribe("ballposition", 1, vision::ballPositionCallback);


  ROS_INFO("start spin now"); 

  ros::spin();

  cv::destroyWindow("3D marker position");
}

/*

	HUMANOID ROBOTIC SYSTEMS
	Final Project

	Control Part

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

// Include customn messages:
#include "RoNAOldo/visionMsg.h"
#include "RoNAOldo/controlMsg.h"

//using namespace aruco;
using namespace cv;
using namespace std;
using namespace ros;

// Thread for spinning:
bool stop_thread=false;

void spinThread()
{
    while(!stop_thread)
    {
        ros::spinOnce();
        //ROS_INFO_STREAM("Spinning the thing!!");
    }
}

// Control Class:
class Control
{
public:

    // ros handler
    ros::NodeHandle nh_;

    // subscriber
    ros::Subscriber controlSub;

    // publisher
    ros::Publisher controlPub;

    // Spin Thread:
    boost::thread *spin_thread;

    // DEBUG MODE:
    bool DEBUG = true;

    // Vision Info:
    float BALL_REL_TO_GOAL;
    float BALL_DIST;
    bool DATA_IS_NEW = false;
    float BALL_REL_TO_IMAGE = 0;

    // Control Paramers:
    bool ORIENTATION_OK = false;
    float STEP_SIZE = 0;
    float ORIENTATION_TOLERANCE = 1;
    float POSITION_TOLERANCE = 1;

    Control(NodeHandle n) {

		nh_ = n;

		controlSub = nh_.subscribe("visionMessage", 10, &Control::visionMessageCallback, this);

		controlPub = nh_.advertise<RoNAOldo::controlMsg>("controlMessage", 10);

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);

  }

    ~Control() {
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }

    void main_loop() {
  		ros::Rate rate_sleep(1);

  		while(nh_.ok())
  		{

        if (DATA_IS_NEW == true) {

          // ----- BEGIN CONTROL ALGORITHM -----

          // Check if Orientation is OK:
          if (false /* Condition for Orientation */) {
            if (true /* Condition for Position*/) {
              if(true /* Condition for Approach*/) {
                if(true /* Condition for Kick */) {
                  controlKick();
                }
              }
              else {
                controlApproach();
              }
            }
            else {
              // SIDESTEP CONTROLLER:
              controlPosition();
            }
          }
          else {
            // ORIENTATION CONTROLLER:
            controlOrientation();
          }

          // ----- END CONTROL ALGORITHM -----

          // DATA_IS_NEW to false:
          DATA_IS_NEW = false;

        }

        // Rate:
        rate_sleep.sleep();

  		}

    }

    // ORIENTATION CONTROLLER:
    void controlOrientation () {

      // Check where ball is relative to image
      if (BALL_REL_TO_IMAGE > 0) {      // right side in image
        // Turn Right
      }
      else {                            // left side in image
        // Turn Left
      }

    }

    // POSITION CONTROLLER:
    void controlPosition() {

      // Calculate stepsize:
      // TODO

      // Check where ball is relative to goal
      if (BALL_REL_TO_GOAL > 0) {       // right of goal
        // Sidestep right
      }
      else {                            // left of goal
        // Sidestep left
      }

    }

    // APPROACHING CONTROLLER:
    void controlApproach() {

      // TODO
      // 1. Control Offset for kicking with right foot
      // 2. Control Approach to ball

    }

    // KICKING CONTROLLER:
    void controlKick() {

      // TODO
      // 1. Stay on one foot
      // 2. Perfrom Kick

    }

    void visionMessageCallback(const RoNAOldo::visionMsg::ConstPtr &inMessage) {

      BALL_REL_TO_GOAL = inMessage->ball_off_middle;
      BALL_DIST = inMessage->ball_distance;
      BALL_REL_TO_IMAGE = inMessage->ball_rel_image;

      DATA_IS_NEW = true;

      if (DEBUG == true) {
        cout << "Received new Data:" << endl;
        cout << "Distance to ball: " << BALL_DIST << endl;
        cout << "Ball position relative to goal: " << BALL_REL_TO_GOAL << endl;
        cout << "Ball position relative to image: " << BALL_REL_TO_IMAGE << endl;
        cout << endl;
      }

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RoNAOldo_control");

    ros::NodeHandle n;

    Control cNode(n);
    cNode.main_loop();

    cout << "Control has terminated!" << endl;

    return 0;

}

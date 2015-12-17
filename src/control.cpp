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
#include <naoqi_bridge_msgs/BodyPoseActionGoal.h>
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
#include <geometry_msgs/Pose2D.h>
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

    // walker publisher:
    ros::Publisher walk_pub;

    // movement status Subscriber:
    ros::Subscriber moveStatus;

    // Spin Thread:
    boost::thread *spin_thread;

    // DEBUG MODE:
    bool DEBUG = true;

    // Vision Info:
    float BALL_REL_TO_GOAL;
    float BALL_DIST;
    bool DATA_IS_NEW = false;
    float BALL_REL_TO_IMAGE = 0;

    // Control Paramers (ORIENTATION CONTROLLER):
    bool ORIENTATION_OK   = false;
    float ORI_ORI_GAIN    = 0;
    float ORI_ORI_TOL     = 0.03;
    float ORI_POS_GAIN    = 0;
    float ORI_POS_TOL     = 0.03;

    // Control Parameters (APPROACH CONTROLLER):
    bool APPROACH_OK     = false;
    //float APP_ORI_GAIN  = 0;
    //float APP_ORI_TOL   = 0.03;
    float APP_POS_GAIN  = 0;
    float APP_POS_TOL   = 0.3;
    //bool BALL_OFFSET_CORRECTED = false;

    // Control Parameters (KICK CONTROLLER):
    // TODO

    Control(NodeHandle n) {

		nh_ = n;

      // Setup Publisher and Subscriber:
  		controlSub = nh_.subscribe("visionMessage", 10, &Control::visionMessageCallback, this);
  		controlPub = nh_.advertise<RoNAOldo::controlMsg>("controlMessage", 10);
      walk_pub = nh_.advertise<geometry_msgs::Pose2D>("/nao/cmd_pose", 1);
      //moveStatus = nh_.subscribe("/nao/joint_angles_status/status", 1, &Control::bodyPoseCallback, this);

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

  		while(nh_.ok()) {

        if (DATA_IS_NEW == true) {

          if (DEBUG == true) {
            cout << "-------------------------------------------------------" << endl;
            cout << "\033[1;31mNew Control Iteration:\033[0m" << endl;
            cout << "    \033[3mBALL_DST            \033[0m : " << setw(10) << BALL_DIST << endl;
            cout << "    \033[3mBALL_REL_TO_GOAL    \033[0m : " << setw(10) << BALL_REL_TO_GOAL << endl;
            cout << "    \033[3mBALL_REL_TO_IMAGE   \033[0m : " << setw(10) << BALL_REL_TO_IMAGE << endl;
            cout << "    \033[3mORIENTATION_OK      \033[0m : " << setw(10) << ORIENTATION_OK << endl;
            cout << "    \033[3mAPPROACH_OK         \033[0m : " << setw(10) << APPROACH_OK << endl;
          }

          // ----- BEGIN CONTROL ALGORITHM -----

          // ORIENTATION CONTROLLER:
          if (ORIENTATION_OK == false && APPROACH_OK == false) {

            // Check if orientation is ok:
            if (BALL_REL_TO_IMAGE >= -ORI_ORI_TOL && BALL_REL_TO_IMAGE <= ORI_ORI_TOL) {

              // Check if position is ok:
              if (BALL_REL_TO_GOAL >= -ORI_POS_TOL && BALL_REL_TO_GOAL <= ORI_POS_TOL) {

                // ORIENTATION CONTROL done:
                ORIENTATION_OK = true;

                if (DEBUG == true) {
                  cout << "\033[1;33mOrientation control is now done!\033[0m" << endl;
                }

              }
              else {

                // Perform ORI-POS-CONTROL:
                controlOriPos();

              }
            }
            else {

              // Perform ORI_ORI_CONTROL:
              controlOri();

            }

          }
          // APPROACH CONTROLLER:
          else if (ORIENTATION_OK == true && APPROACH_OK == false) {

            // Check if orientation is ok:
            if (BALL_REL_TO_IMAGE >= -ORI_ORI_TOL && BALL_REL_TO_IMAGE <= ORI_ORI_TOL) {

              // Check if position is ok:
              if (BALL_DIST < APP_POS_TOL) {

                // APPROACH CONTROL done:
                APPROACH_OK = true;

                if (DEBUG == true) {
                  cout << "\033[1;33mApproach control is now done!\033[0m" << endl;
                }

              }
              else {

                // Perform ORI-POS-CONTROL:
                controlAppPos();

              }
            }
            else {

              // Perform ORI_ORI_CONTROL:
              controlOri();

            }

          }
          // KICK CONTROLLER:
          else if (ORIENTATION_OK == true && APPROACH_OK == true) {

            // Perform Kick:
            controlKick();

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
    void controlOri() {

      // calculate gain
      ORI_ORI_GAIN = abs(BALL_REL_TO_IMAGE) / 1.0;

      // Check where ball is relative to image
      if (BALL_REL_TO_IMAGE > 0) {      // right side in image
        if (DEBUG == true) {
          cout << "\033[1;33mPerforming Orientation Control:\033[0m" << endl;
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_TO_IMAGE > 0)" << endl;
          cout << "    \033[3mORIENTATION_GAIN     \033[0m: " << setw(10) << ORI_ORI_GAIN  << endl;
        }
        walker(0, 0, -ORI_ORI_GAIN);
      }
      else {                            // left side in image
        if (DEBUG == true) {
          cout << "\033[1;33mPerforming Orientation Control:\033[0m" << endl;
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_TO_IMAGE < 0)" << endl;
          cout << "    \033[3mORIENTATION_GAIN     \033[0m: " << setw(10) << ORI_ORI_GAIN  << endl;
        }
        walker(0, 0, ORI_ORI_GAIN);
      }

    }

    // POSITION CONTROLLER:
    void controlOriPos() {

      // Calculate stepsize:
      ORI_POS_GAIN = abs(BALL_REL_TO_GOAL) / 5.0;

      // Check where ball is relative to goal
      if (BALL_REL_TO_GOAL > 0) {       // right of goal
        if (DEBUG == true) {
          cout << "\033[1;33mPerforming Position Control:\033[0m" << endl;
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_To_IMAGE > 0)" << endl;
          cout << "    \033[3mPOSITION_GAIN        \033[0m: " << setw(10) << ORI_POS_GAIN  << endl;
        }
        walker(0, -ORI_POS_GAIN, 0);
      }
      else {                            // left of goal
        if (DEBUG == true) {
          cout << "\033[1;33mPerforming Position Control:\033[0m" << endl;
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_To_IMAGE < 0)" << endl;
          cout << "    \033[3mPOSITION_GAIN        \033[0m: " << setw(10) << ORI_POS_GAIN  << endl;
        }
        walker(0, ORI_POS_GAIN, 0);
      }

    }

    // APPROACHING CONTROLLER:
    void controlAppPos() {

      if (DEBUG == true) {
        cout << "\033[1;33mPerforming Approach Control: \033[0m" << endl;
      }

      // Correct ball offset if not already done:
      //if (BALL_OFFSET_CORRECTED == false) {

      //  if (DEBUG == true) {
      //    cout << "Correcting ball offset\033[0m" << endl;
      //  }

      // Walk to the left:
      //  walker(0, 0.05, 0);

      // Set BALL_OFFSET_CORRECTED to true:
      //  BALL_OFFSET_CORRECTED = true;

      //}
      // Approach control:
      //else {

      // Approach gain:
      APP_POS_GAIN = BALL_DIST * 0.2;

      // Approach the ball:
      walker(APP_POS_GAIN, 0, 0);

      //}

    }

    // KICKING CONTROLLER:
    void controlKick() {

      if (DEBUG == true) {
        cout << "\033[1;33mPerforming Kick Control:\033[0m" << endl;
      }

      // TODO
      // 1. Stay on one foot
      // 2. Perfrom Kick
      walker(0, 0.05, 0);
      walker(0.2, 0, 0);

    }

    // Walker function:
    void walker(double x, double y, double theta) {

      if (DEBUG == true) {
        cout << "\033[1;34mMoving:\033[0m" << endl;
        cout << "    \033[3mX                   \033[0m : " << setw(10) << x << endl;
        cout << "    \033[3mY                   \033[0m : " << setw(10) << y << endl;
        cout << "    \033[3mtheta               \033[0m : " << setw(10) << theta << endl;
      }

  		// Create Message:
  		geometry_msgs::Pose2D pose;
  		pose.x = x;
  		pose.y = y;
  		pose.theta = theta;

  		// Publish Message:
  		walk_pub.publish(pose);

      // Wait until movement is finished:
      sleep(5);

      //while( //movement still active )
      //{
      //
      //}
      // rostopic list
      // rostopic info /nao/...

      if (DEBUG == true) {
        cout << "\033[1;32mMovement done!\033[0m" << endl;
      }

  	}

    void visionMessageCallback(const RoNAOldo::visionMsg::ConstPtr &inMessage) {

      BALL_REL_TO_GOAL = inMessage->ball_rel_goal;
      BALL_DIST = inMessage->ball_distance;
      BALL_REL_TO_IMAGE = inMessage->ball_rel_image;

      DATA_IS_NEW = true;

    }

    //void bodyPoseCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &inMessage) {

        //cout << "Pose status size: " << inMessage->status_list.size() << endl;
        //if(!inMessage->status_list.empty()) {
        //  cout << "Pose Status: " << inMessage->status_list.at(0).status; // << inMessage->goal << endl;
        //}

    //}

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "RoNAOldo_control");

    ros::NodeHandle n;

    Control cNode(n);
    cNode.main_loop();

    cout << "Control has terminated!" << endl;

    return 0;

}

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
#include <unistd.h>
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
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
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

// State Machine:
#define STATE_INIT                0
#define STATE_BALL_SEARCH         1
#define STATE_LEFT_FOUND          2
#define STATE_RIGHT_FOUND         3
#define STATE_STEP_BACK_LEFT      4
#define STATE_STEP_BACK_RIGHT     5
#define STATE_CONTROL             6
#define STATE_GOAL_SEARCH         7
#define STATE_ERROR               9

// Extended bools:
#define F     0 // TRUE
#define T     1 // FALSE
#define X     2 // DONT CARE

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
    //ros::Subscriber moveStatus;

    // Spin Thread:
    boost::thread *spin_thread;

    // DEBUG MODE:
    bool DEBUG = true;
    int iterCount = 0;

    // State Machine:
    bool FINISHED = false;
    int STATE = STATE_INIT;
    bool TRANS[3] = {false};

    // Vision Info:
    float BALL_REL_TO_GOAL = 0;
    float BALL_DIST = 0;
    bool DATA_IS_NEW = false;
    float BALL_REL_TO_IMAGE = 0;
    float BALL_REL_TO_IMAGE_Y = 0;
    bool BALL_VISIBLE = false;
    bool LEFT_POST_VISIBLE = false;
    bool RIGHT_POST_VISIBLE = false;
    int BALL_TOP_BOTTOM_CAMERA = 0;

    // Control Paramers (ORIENTATION CONTROLLER):
    bool ORIENTATION_OK   = false;
    float ORI_ORI_GAIN    = 0;
    float ORI_ORI_TOL     = 0.1;
    float ORI_POS_GAIN    = 0;
    float ORI_POS_TOL     = 0.1;

    // Control Parameters (APPROACH CONTROLLER):
    bool APPROACH_OK     = false;
    //float APP_ORI_GAIN  = 0;
    //float APP_ORI_TOL   = 0.03;
    float APP_POS_GAIN  = 0;
    float APP_POS_TOL   = 0.3;
    //bool BALL_OFFSET_CORRECTED = false;
    bool APP_HEAD_DOWN = false;

    // Control Parameters (KICK CONTROLLER):
    // TODO

    // Joint States:
    sensor_msgs::JointState LEFT_ARM_STATE;
    sensor_msgs::JointState RIGHT_ARM_STATE;
    sensor_msgs::JointState HEAD_LEGS_STATE;
    ros::Subscriber jointStatesSub;
    ros::Subscriber jointActionStatusSub;
    ros::Publisher jointCommandPub;
    int JOINT_ACTION_STATUS = 0;

    // Speech:
    ros::Publisher speechPub;

    Control(NodeHandle n) {

		  nh_ = n;

      // Setup Publisher and Subscriber:
  		controlSub = nh_.subscribe("relative_position", 1, &Control::visionMessageCallback, this);
  		controlPub = nh_.advertise<RoNAOldo::controlMsg>("controlMessage", 1);
      walk_pub = nh_.advertise<geometry_msgs::Pose2D>("/nao/cmd_pose", 1);
      speechPub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/nao/speech_action/goal", 1);
      //moveStatus = nh_.subscribe("/nao/joint_angles_status/status", 1, &Control::bodyPoseCallback, this);
      jointStatesSub = nh_.subscribe("/nao/joint_states", 1, &Control::jointStatusCallback, this);
      jointActionStatusSub = nh_.subscribe("/nao/joint_angles_action/status",1, &Control::jointActionStatusCallback, this);
      jointCommandPub = nh_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal>("/nao/joint_angles_action/goal", 1);

      // To keep the node alive:
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

  		while(nh_.ok() && FINISHED == false) {

        if (DATA_IS_NEW == true) {

        // Count Iteration:
        iterCount++;

          if (DEBUG == true) {
            cout << "-------------------------------------------------------" << endl;
            cout << "\033[1;34mNew Iteration:\033[0m #" << setw(10) << iterCount << endl;
            cout << endl;
            cout << "\033[1;34mCurent State:\033[0m " << stateToString(STATE) << endl;
            cout << "\033[1;36mCurrent Values:\033[0m" << endl;
            cout << "    \033[3mBALL_DST            \033[0m : " << setw(10) << BALL_DIST << endl;
            cout << "    \033[3mBALL_REL_TO_GOAL    \033[0m : " << setw(10) << BALL_REL_TO_GOAL << endl;
            cout << "    \033[3mBALL_REL_TO_IMAGE   \033[0m : " << setw(10) << BALL_REL_TO_IMAGE << endl;
            cout << "    \033[3mBALL_REL_TO_IMAGE_Y \033[0m : " << setw(10) << BALL_REL_TO_IMAGE_Y << endl;
            cout << "\033[1;36mCurrent Logic Values:\033[0m" << endl;
            cout << "    \033[3mBALL_VISIBLE        \033[0m : " << setw(10) << boolToString(BALL_VISIBLE) << endl;
            cout << "    \033[3mLEFT_POST_VISIBLE   \033[0m : " << setw(10) << boolToString(LEFT_POST_VISIBLE) << endl;
            cout << "    \033[3mRIGHT_POST_VISIBLE  \033[0m : " << setw(10) << boolToString(RIGHT_POST_VISIBLE) << endl;
            cout << "    \033[3mORIENTATION_OK      \033[0m : " << setw(10) << boolToString(ORIENTATION_OK) << endl;
            cout << "    \033[3mAPPROACH_OK         \033[0m : " << setw(10) << boolToString(APPROACH_OK) << endl;
          }

          // ----- BEGIN STATE MACHINE -----

          // Update Transition Vector:
          TRANS[0] = BALL_VISIBLE;
          TRANS[1] = LEFT_POST_VISIBLE;
          TRANS[2] = RIGHT_POST_VISIBLE;

          // Determine Current state:
          switch(STATE) {

            case STATE_INIT:
            init();
              if (boolCompare(TRANS, T, F, F)) {
                transState(STATE_GOAL_SEARCH);
              }
              else if (boolCompare(TRANS, T, F, T)) {
                transState(STATE_RIGHT_FOUND);
              }
              else if (boolCompare(TRANS, T, T, F)) {
                transState(STATE_LEFT_FOUND);
              }
              else if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else {
                transState(STATE_BALL_SEARCH);
              }
              break;

            case STATE_BALL_SEARCH:
              if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else if (boolCompare(TRANS, F, X, X)) {
                transState(STATE_BALL_SEARCH);
              }
              else if (boolCompare(TRANS, T, F, F)) {
                transState(STATE_GOAL_SEARCH);
              }
              else if (boolCompare(TRANS, T, F, T)) {
                transState(STATE_RIGHT_FOUND);
              }
              else if (boolCompare(TRANS, T, T, F)) {
                transState(STATE_LEFT_FOUND);
              }
              else {
                transState(STATE_BALL_SEARCH);
              }
              break;

            case STATE_LEFT_FOUND:
              if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else if (boolCompare(TRANS, T, F, T)) {
                transState(STATE_STEP_BACK_LEFT);
              }
              else if (boolCompare(TRANS, F, X, T)) {
                transState(STATE_STEP_BACK_LEFT);
              }
              else if (boolCompare(TRANS, X, X, F)) {
                transState(STATE_LEFT_FOUND);
              }
              else {
                transState(STATE_LEFT_FOUND);
              }
              break;

            case STATE_RIGHT_FOUND:
              if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else if (boolCompare(TRANS, T, T, F)) {
                transState(STATE_STEP_BACK_RIGHT);
              }
              else if (boolCompare(TRANS, F, T, X)) {
                transState(STATE_STEP_BACK_RIGHT);
              }
              else if (boolCompare(TRANS, X, F, X)) {
                transState(STATE_RIGHT_FOUND);
              }
              else {
                transState(STATE_RIGHT_FOUND);
              }
              break;
            break;

            case STATE_STEP_BACK_LEFT:
              if (boolCompare(TRANS, T, F, T)) {
                transState(STATE_RIGHT_FOUND);
              }
              else if (boolCompare(TRANS, F, X, X)) {
                transState(STATE_BALL_SEARCH);
              }
              else if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else {
                transState(STATE_STEP_BACK_LEFT);
              }
              break;

            case STATE_STEP_BACK_RIGHT:
              if (boolCompare(TRANS, T, T, F)) {
                transState(STATE_LEFT_FOUND);
              }
              else if (boolCompare(TRANS, F, X, X)) {
                transState(STATE_BALL_SEARCH);
              }
              else if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else {
                transState(STATE_STEP_BACK_LEFT);
              }
              break;

            case STATE_CONTROL:
              if (boolCompare(TRANS, F, X, X)) {
                transState(STATE_BALL_SEARCH);
              }
              else if (boolCompare(TRANS, T, X, X)) {
                transState(STATE_CONTROL);
              }
              else {
                transState(STATE_CONTROL);
              }
              break;

            case STATE_GOAL_SEARCH:
              if (boolCompare(TRANS, T, T, T)) {
                transState(STATE_CONTROL);
              }
              else if (boolCompare(TRANS, T, F, T)) {
                transState(STATE_RIGHT_FOUND);
              }
              else if (boolCompare(TRANS, T, T, F)) {
                transState(STATE_LEFT_FOUND);
              }
              else if (boolCompare(TRANS, F, X, X)) {
                transState(STATE_BALL_SEARCH);
              }
              else {
                walker(0, 0, 1);
                transState(STATE_GOAL_SEARCH);
              }
              break;

            case STATE_ERROR:
              break;

          }

          // ----- END STATE MACHINE -----

          // DATA_IS_NEW to false:
          DATA_IS_NEW = false;

        }

        // Rate:
        rate_sleep.sleep();

  		}

    }

    // Bool Comparison:
    bool boolCompare(bool trans[3], int s0, int s1, int s2) {

      bool out = true;

      // s0
      switch(s0) {
        case F:
          if (trans[0] != false) {
            out = false;
          }
          break;
        case T:
          if (trans[0] != true) {
            out = false;
          }
          break;
        case X:
          break;
      }

      // s1
      switch(s1) {
        case F:
          if (trans[1] != false) {
            out = false;
          }
          break;
        case T:
          if (trans[1] != true) {
            out = false;
          }
          break;
        case X:
          break;
      }

      // s2
      switch(s2) {
        case F:
          if (trans[2] != false) {
            out = false;
          }
          break;
        case T:
          if (trans[2] != true) {
            out = false;
          }
          break;
        case X:
          break;
      }

      return out;

    }

    // State transition function with debug information:
    void transState(int nextState) {
      if (DEBUG == true) {
        cout << "\033[1;33mState Transition:\033[0m " << stateToString(STATE) << " --> " << stateToString(nextState) << endl;
      }

      // To State Action:
      switch (nextState) {
          case STATE_INIT:
            init();
            break;
          case STATE_CONTROL:
            control();
            break;
          case STATE_BALL_SEARCH:
            ballSearch();
            break;
          case STATE_RIGHT_FOUND:
            rightFound();
            break;
          case STATE_LEFT_FOUND:
            leftFound();
            break;
          case STATE_STEP_BACK_RIGHT:
            stepBack();
            break;
          case STATE_STEP_BACK_LEFT:
            stepBack();
            break;
          case STATE_GOAL_SEARCH:
            goalSearch();
            break;
          case STATE_ERROR:
            cout << "\033[1;31mHELP I'M IN ERROR STATE!\033[0m" << endl;
            break;
      }
      STATE = nextState;
    }

    // State to string conversion:
    string stateToString(int state) {

      switch (state) {
        case STATE_INIT:
          return "STATE_INIT";
        case STATE_CONTROL:
          return "STATE_CONTROL";
        case STATE_BALL_SEARCH:
          return "STATE_BALL_SEARCH";
        case STATE_RIGHT_FOUND:
          return "STATE_RIGHT_FOUND";
        case STATE_LEFT_FOUND:
          return "STATE_LEFT_FOUND";
        case STATE_STEP_BACK_RIGHT:
          return "STATE_STEP_BACK_RIGHT";
        case STATE_STEP_BACK_LEFT:
          return "STATE_STEP_BACK_LEFT";
        case STATE_GOAL_SEARCH:
          return "STATE_GOAL_SEARCH";
        case STATE_ERROR:
          return "STATE_ERROR";
      }

    }

    string boolToString(bool b) {
      if (b == true) {
        return "TRUE";
      }
      else {
        return "FALSE";
      }
    }

    // STATE -> CONTROL:
    void control() {
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

          if (DEBUG == true) {
            //cout << "\033[1;33mHead: \033[0m" << APP_STATE << endl;
          }
          if (/*BALL_TOP_BOTTOM_CAMERA == 1 &&*/ BALL_REL_TO_IMAGE_Y < 0.5f && APP_HEAD_DOWN == false) {

            controlAppPos(0.2);

          }
          else if (/*BALL_TOP_BOTTOM_CAMERA == 1 &&*/ BALL_REL_TO_IMAGE_Y >= 0.5f && APP_HEAD_DOWN == false) {

            // Turn Head down:
            sensor_msgs::JointState turnHead;
            turnHead.name.push_back("HeadPitch");
            turnHead.position.push_back(0.375);
            mooveJoints(turnHead, 0.1, 1);

            // Set APP_HEAD_DOWN to true:
            APP_HEAD_DOWN = true;

          }
          else if (/*BALL_TOP_BOTTOM_CAMERA == 1 &&*/ BALL_REL_TO_IMAGE_Y < 0.3f && APP_HEAD_DOWN == true) {

            controlAppPos(0.05);

          }
          else if (/*BALL_TOP_BOTTOM_CAMERA == 1 &&*/ BALL_REL_TO_IMAGE_Y < 0.55f && APP_HEAD_DOWN == true) {

            walker(0.01, 0, 0);

          }
          else if (/*BALL_TOP_BOTTOM_CAMERA == 1 &&*/ BALL_REL_TO_IMAGE_Y >= 0.55f && APP_HEAD_DOWN == true) {

            // Approach done:
            APPROACH_OK = true;

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

        // Terminate Node:
        FINISHED == true;

        // Goal cheer:
        speak("ro NAO eldo scored.");
        sensor_msgs::JointState cheer;
        cheer.name.push_back("LShoulderPitch");
        cheer.position.push_back(-1.6);
        cheer.name.push_back("RShoulderPitch");
        cheer.position.push_back(-1.6);
        mooveJoints(cheer, 0.3, 0);

        // Exit:
        std::exit(0);

      }

    }

    // ORIENTATION CONTROLLER:
    void controlOri() {

      // calculate gain
      ORI_ORI_GAIN = abs(BALL_REL_TO_IMAGE) / 1.25;

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
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_TO_IMAGE > 0)" << endl;
          cout << "    \033[3mPOSITION_GAIN        \033[0m: " << setw(10) << ORI_POS_GAIN  << endl;
        }
        walker(0, -ORI_POS_GAIN, 0);
      }
      else {                            // left of goal
        if (DEBUG == true) {
          cout << "\033[1;33mPerforming Position Control:\033[0m" << endl;
          cout << "    \033[3mcondition            \033[0m: " << " (BALL_REL_TO_IMAGE < 0)" << endl;
          cout << "    \033[3mPOSITION_GAIN        \033[0m: " << setw(10) << ORI_POS_GAIN  << endl;
        }
        walker(0, ORI_POS_GAIN, 0);
      }

    }

    // APPROACHING CONTROLLER:
    void controlAppPos(double gainMult) {

      if (DEBUG == true) {
        cout << "\033[1;33mPerforming Approach Control: \033[0m" << endl;
      }

      // Approach gain:
      APP_POS_GAIN = BALL_DIST * 0.2;

      // Approach the ball:
      walker(APP_POS_GAIN, 0, 0);

      //}

    }

    // Ball search:
    void ballSearch() {

      // Turn left:
      walker(0, 0, 1);

    }

    // Left Found:
    void leftFound() {

      // Turn right:
      walker(0, 0, -0.3);

    }

    // Right Found:
    void rightFound() {

      // Turn left:
      walker(0, 0, 0.3);

    }

    // Step Back:
    void stepBack() {

      // Step Back:
      walker(-0.5, 0, 0);

    }

    // Init:
    void init() {

      // Stand up:
      walker(0.01, 0, 0);

      // Align Head:
      sensor_msgs::JointState alignHead;
      alignHead.name.push_back("HeadYaw");
      alignHead.name.push_back("HeadPitch");
      alignHead.position.push_back(0.0);
      alignHead.position.push_back(0.0);
      mooveJoints(alignHead, 0.2, 0);
      sleep(1);

      // Say Hello:
      speak("hello my name is ro NAO eldo");

    }

    void goalSearch() {

      // Align Ball relative to image:
      controlOri();

      // Init Head turning:
      float angle = 0.0;
      bool GOAL_FOUND = false;
      sensor_msgs::JointState turnHead;
      turnHead.name.push_back("HeadYaw");
      turnHead.position.push_back(0.0);

      // Turn Head left:
      for (int i = 1; i <= 5; i++) {

        // Turn Head: (20deg per section)
        angle = i * 0.35;
        turnHead.position[0] = angle;
        mooveJoints(turnHead, 0.05, 0);
        sleep(1);

        // Check if right goal post found:
        DATA_IS_NEW = false;
        while(!DATA_IS_NEW) {}
        if(RIGHT_POST_VISIBLE == true) {
          GOAL_FOUND = true;
          goto POSTFOUND;
        }

      }

      // Turn Head Back:
      turnHead.position[0] = 0.0;
      mooveJoints(turnHead, 0.2, 0);
      sleep(1);

      // Turn Head right:
      for (int i = 1; i <= 5; i++) {

        // Turn Head: (20deg per section)
        angle = i * -0.35;
        turnHead.position[0] = angle;
        mooveJoints(turnHead, 0.05, 0);
        sleep(1);

        // Check if left goal post found:
        DATA_IS_NEW = false;
        while(!DATA_IS_NEW) {}
        if(LEFT_POST_VISIBLE == true) {
          GOAL_FOUND = true;
          goto POSTFOUND;
        }

      }

      POSTFOUND:

      // Turn Head Back:
      turnHead.position[0] = 0.0;
      mooveJoints(turnHead, 0.2, 0);
      sleep(1);

      // Move Robot if goal found:
      if (GOAL_FOUND == true) {
        walker(0, 0, angle);
        if (angle > 0) {
          walker(0, -0.5, 0);
        }
        else {
          walker(0, 0.5, 0);
        }
      }

      // Orientation Control:
      DATA_IS_NEW = false;
      while(!DATA_IS_NEW) {}
      controlOri();

    }

    // KICKING CONTROLLER:
    void controlKick() {

      if (DEBUG == true) {
        cout << "\033[1;33mPerforming Kick Control:\033[0m" << endl;
      }

      // Go 5cm left:
      walker(0, 0.05, 0);
      sleep(2);

      // Lean zo left:
      sensor_msgs::JointState oneFoot1;
      oneFoot1.name.push_back("LKneePitch");
      oneFoot1.name.push_back("LAnklePitch");
      oneFoot1.name.push_back("RKneePitch");
      oneFoot1.name.push_back("RAnklePitch");
      oneFoot1.name.push_back("LAnkleRoll");
      oneFoot1.position.push_back(0.2);
      oneFoot1.position.push_back(-0.1);
      oneFoot1.position.push_back(-0.2);
      oneFoot1.position.push_back(0.2);
      oneFoot1.position.push_back(0.2);
      mooveJoints(oneFoot1, 0.05, 1);

      // stand on one foot:
      sensor_msgs::JointState oneFoot2;
      oneFoot2.name.push_back("RKneePitch");
      oneFoot2.name.push_back("RAnklePitch");
      oneFoot2.position.push_back(0.4);
      oneFoot2.position.push_back(-0.4);
      mooveJoints(oneFoot2, 0.05, 1);

      // kick:
      sensor_msgs::JointState oneFoot3;
      oneFoot3.name.empty();
      oneFoot3.position.empty();
      oneFoot3.name.push_back("RHipPitch");
      oneFoot3.name.push_back("RAnklePitch");
      oneFoot3.position.push_back(-0.5);
      oneFoot3.position.push_back(0.4);
      mooveJoints(oneFoot3, 0.8, 1);

      // retract foot:
      sensor_msgs::JointState oneFoot4;
      oneFoot4.name.empty();
      oneFoot4.position.empty();
      oneFoot4.name.push_back("RHipPitch");
      oneFoot4.name.push_back("RAnklePitch");
      oneFoot4.position.push_back(0.5);
      oneFoot4.position.push_back(-0.4);
      mooveJoints(oneFoot4, 0.2, 1);

      // stand on both foot:
      sensor_msgs::JointState oneFoot5;
      oneFoot5.name.push_back("RKneePitch");
      oneFoot5.name.push_back("RAnklePitch");
      oneFoot5.position.push_back(-0.4);
      oneFoot5.position.push_back(0.4);
      mooveJoints(oneFoot5, 0.05, 1);

      // Lean back to neutral position:
      sensor_msgs::JointState oneFoot6;
      oneFoot6.name.push_back("LKneePitch");
      oneFoot6.name.push_back("LAnklePitch");
      oneFoot6.name.push_back("RKneePitch");
      oneFoot6.name.push_back("RAnklePitch");
      oneFoot6.name.push_back("LAnkleRoll");
      oneFoot6.position.push_back(-0.2);
      oneFoot6.position.push_back(0.1);
      oneFoot6.position.push_back(0.2);
      oneFoot6.position.push_back(-0.2);
      oneFoot6.position.push_back(-0.2);
      mooveJoints(oneFoot6, 0.05, 1);

    }

    // Walker function:
    void walker(double x, double y, double theta) {

      if (DEBUG == true) {
        cout << "\033[1;33mMoving:\033[0m" << endl;
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
      float sleepTime = 0;
      sleepTime += abs(theta * 5.0);
      sleepTime += abs(x * 10.0);
      sleepTime += abs(y * 20.0);
      cout << "\033[1;33mWaiting: \033[0m" << max(2,(int)sleepTime) << "\033[1;33m seconds\033[0m" << endl;
      sleep(max(2,(int)sleepTime));
      // TODO: Find a better method
      if (DEBUG == true) {
        cout << "\033[1;32mMovement done!\033[0m" << endl;
      }

  	}

    void visionMessageCallback(const RoNAOldo::visionMsg::ConstPtr &inMessage) {

      if (inMessage->ball_detected_in_lastsec) {

        BALL_REL_TO_IMAGE = inMessage->ball_rel_image_x;
        BALL_REL_TO_IMAGE_Y = inMessage->ball_rel_image_y;
        BALL_DIST = inMessage->ball_distance;
        BALL_TOP_BOTTOM_CAMERA = inMessage->ball_in_top_or_bottom_camera;

        if (inMessage->left_marker_detected_in_lastsec && inMessage->right_marker_detected_in_lastsec) {

          BALL_REL_TO_GOAL = inMessage->ball_rel_goal;

        }

      }

      BALL_VISIBLE = inMessage->ball_detected_in_lastsec;
      LEFT_POST_VISIBLE = inMessage->left_marker_detected_in_lastsec;
      RIGHT_POST_VISIBLE = inMessage->right_marker_detected_in_lastsec;

      DATA_IS_NEW = true;

    }

    void jointStatusCallback(const sensor_msgs::JointState::ConstPtr &jointState) {

      LEFT_ARM_STATE.name.clear();
      LEFT_ARM_STATE.position.clear();
      RIGHT_ARM_STATE.name.clear();
      RIGHT_ARM_STATE.position.clear();
      HEAD_LEGS_STATE.name.clear();
      HEAD_LEGS_STATE.position.clear();

      LEFT_ARM_STATE.header.stamp=ros::Time::now();

      LEFT_ARM_STATE.name.push_back(jointState->name.at(2));
      LEFT_ARM_STATE.position.push_back(jointState->position.at(2));
      LEFT_ARM_STATE.name.push_back(jointState->name.at(3));
      LEFT_ARM_STATE.position.push_back(jointState->position.at(3));
      LEFT_ARM_STATE.name.push_back(jointState->name.at(4));
      LEFT_ARM_STATE.position.push_back(jointState->position.at(4));
      LEFT_ARM_STATE.name.push_back(jointState->name.at(5));
      LEFT_ARM_STATE.position.push_back(jointState->position.at(5));
      LEFT_ARM_STATE.name.push_back(jointState->name.at(6));
      LEFT_ARM_STATE.position.push_back(jointState->position.at(6));

      RIGHT_ARM_STATE.name.push_back(jointState->name.at(20));
      RIGHT_ARM_STATE.position.push_back(jointState->position.at(20));
      RIGHT_ARM_STATE.name.push_back(jointState->name.at(21));
      RIGHT_ARM_STATE.position.push_back(jointState->position.at(21));
      RIGHT_ARM_STATE.name.push_back(jointState->name.at(22));
      RIGHT_ARM_STATE.position.push_back(jointState->position.at(22));
      RIGHT_ARM_STATE.name.push_back(jointState->name.at(23));
      RIGHT_ARM_STATE.position.push_back(jointState->position.at(23));
      RIGHT_ARM_STATE.name.push_back(jointState->name.at(24));
      RIGHT_ARM_STATE.position.push_back(jointState->position.at(24));

      HEAD_LEGS_STATE.name.push_back(jointState->name.at(0));
      HEAD_LEGS_STATE.position.push_back(jointState->position.at(0));
      HEAD_LEGS_STATE.name.push_back(jointState->name.at(1));
      HEAD_LEGS_STATE.position.push_back(jointState->position.at(1));
      for(int i=7; i<20;i++)
      {
          HEAD_LEGS_STATE.name.push_back(jointState->name.at(i));
          HEAD_LEGS_STATE.position.push_back(jointState->position.at(i));
          //cout << i << " " << jointState->name.at(i) << ": " << jointState->position.at(i) << endl;
      }
      HEAD_LEGS_STATE.name.push_back(jointState->name.at(25));
      HEAD_LEGS_STATE.position.push_back(jointState->position.at(25));
      //cout << jointState->position.at(13) << endl; //LAnkleRoll

    }

    void mooveJoints(sensor_msgs::JointState joints, float speed, int relative) {

      naoqi_bridge_msgs::JointAnglesWithSpeedActionGoal action_execute;
			stringstream ss;
			ss << ros::Time::now().sec;
			action_execute.goal_id.id = "move_" + ss.str();
			action_execute.goal.joint_angles.speed = speed;
			action_execute.goal.joint_angles.relative = relative;
			action_execute.goal.joint_angles.joint_names = joints.name;
      for (int i = 0; i < joints.position.size(); i++) {
        action_execute.goal.joint_angles.joint_angles.push_back((float)joints.position.at(i));
      }
      action_execute.header.stamp = ros::Time::now();
      jointCommandPub.publish(action_execute);
      //JOINT_ACTION_STATUS = 0;
      //usleep(500000);
      sleep(1);
      while((JOINT_ACTION_STATUS == 1 || JOINT_ACTION_STATUS == 0) && nh_.ok()) {}
      sleep(1);

    }

    void jointActionStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
		  if(!msg->status_list.empty()) {
			  JOINT_ACTION_STATUS = (int)msg->status_list.at(0).status;
		  }
    }

    void speak(string sentence) {

      naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speakMsg;
      speakMsg.goal_id.id = "Speak-" + to_string(iterCount);
      speakMsg.goal.say = sentence;
      speechPub.publish(speakMsg);

    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "RoNAOldo_control");

    ros::NodeHandle n;

    Control cNode(n);
    cNode.main_loop();

    cout << "Control has terminated!" << endl;

    return 0;

}

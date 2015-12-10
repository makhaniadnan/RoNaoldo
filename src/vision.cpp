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

    // Spin Thread:
    boost::thread *spin_thread;

    // Counter: <just for testing>
    int count;

    Vision(NodeHandle n) {

		nh_ = n;

		visionSub = nh_.subscribe("controlMessage", 10, &Vision::controlMessageCallback, this);

		visionPub = nh_.advertise<RoNAOldo::visionMsg>("visionMessage", 10);

		count = 0;

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);

    }

   ~Vision() {
        stop_thread=true;
        sleep(1);
        spin_thread->join();
    }

    void main_loop() {

      RoNAOldo::visionMsg msg;
      int count = 0;
      ros::Rate rate_sleep(10);
      while(nh_.ok())
      {
        msg.ballArea = count;
        count++;
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
    ros::init(argc, argv, "RoNAOldo_vision");

    ros::NodeHandle n;

    Vision vNode(n);
    vNode.main_loop();

    cout << "Vision has terminated!" << endl;

    return 0;

}

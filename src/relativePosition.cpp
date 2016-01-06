/*

 HUMANOID ROBOTIC SYSTEMS
 Final Project

 Vision Part

 Group B

 Adnan Makhani
 Andreas Plieninger
 Sebastian Wagner
 Tim Bruedigam


Vision part by
(c) Adnan Makhani
(c) Andreas Plieninger
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

//include custom messages
#include "RoNAOldo/goalPositionMsg.h"
#include "RoNAOldo/ballMsg.h"
#include "RoNAOldo/visionMsg.h"

#include <iostream>
using namespace std;

namespace vision {
class Relative {
public:

	ros::NodeHandle nh_;

	ros::Publisher visionPublisher;
	RoNAOldo::visionMsg v_msg;
	RoNAOldo::ballMsg last_ball_msg;
	RoNAOldo::goalPositionMsg last_goal_msg;

	//position
	//0 top
	//1 bottom
	uint8_t last_ball_msg_position;

	ros::Time last_ball_msg_time;
	ros::Time last_goal_msg_left_time;
	ros::Time last_goal_msg_right_time;

	ros::Subscriber goalSub_top;
	ros::Subscriber goalSub_bottom;
	ros::Subscriber ballSub_top;
	ros::Subscriber ballSub_bottom;

	Relative(ros::NodeHandle n) {
		nh_ = n;

		//publish ballOffMiddle
		visionPublisher = nh_.advertise<RoNAOldo::visionMsg>(
				"relative_position", 10);
		goalSub_top = nh_.subscribe("goalPositionTop", 10, &Relative::goalStore,
				this);
		ballSub_top = nh_.subscribe("ballTop", 10, &Relative::ballTop, this);
		goalSub_bottom = nh_.subscribe("goalPositionBottom", 10,
				&Relative::goalStore, this);
		ballSub_bottom = nh_.subscribe("ballBottom", 10, &Relative::ballBottom,
				this);

		//now minus ten seconds
		last_ball_msg_time = ros::Time::now() - ros::Duration(10);
		last_goal_msg_left_time = ros::Time::now() - ros::Duration(10);
		last_goal_msg_right_time = ros::Time::now() - ros::Duration(10);

		last_ball_msg_position = 0;

		ROS_INFO("Relative class setup done");
	}

	~Relative() {

	}

	void goalStore(RoNAOldo::goalPositionMsg msg) {
		ROS_INFO("store marker");
		last_goal_msg = msg;
		if (msg.marker1_center_x > 0) {
			ROS_INFO("left marker seen");
			last_goal_msg_left_time = ros::Time::now();
		}
		if (msg.marker2_center_x > 0) {
			ROS_INFO("right marker seen");
			last_goal_msg_right_time = ros::Time::now();
		}

		doCalc();
	}
	void ballBottom(RoNAOldo::ballMsg msg) {
		//set to bottom
		last_ball_msg_position = 1;
		cout << "ball in bottom" << endl;
		ballStore(msg);
	}
	void ballTop(RoNAOldo::ballMsg msg) {
		//set to top
		last_ball_msg_position = 0;
		cout << "ball in top" << endl;
		ballStore(msg);
	}
	void ballStore(RoNAOldo::ballMsg msg) {
		ROS_INFO("store ball");
		last_ball_msg = msg;
		last_ball_msg_time = ros::Time::now();
		doCalc();
	}
	void doCalc() {
		RoNAOldo::visionMsg msg;

		msg.ball_detected_in_lastsec = false;
		msg.left_marker_detected_in_lastsec = false;
		msg.right_marker_detected_in_lastsec = false;
		msg.ball_rel_goal = 0;
		msg.ball_rel_image_x = 0;
		msg.ball_rel_image_y = 0;
		msg.ball_distance = 0;
		msg.ball_in_top_or_bottom_camera = 0;

		cout << "last_time" << last_ball_msg_time << endl;
		cout << "now" << ros::Time::now() << endl;
		cout << "duration 1" << ros::Duration(1) << endl;
		ros::Duration diff = (last_ball_msg_time - ros::Time::now());
		cout << "diff" << ros::Duration(1) << endl;

		if ((ros::Time::now() - last_ball_msg_time) < ros::Duration(1)) {
			msg.ball_detected_in_lastsec = true;
		}
		if ((ros::Time::now() - last_goal_msg_left_time) < ros::Duration(1)) {
			msg.left_marker_detected_in_lastsec = true;
		}
		if ((ros::Time::now() - last_goal_msg_right_time) < ros::Duration(1)) {
			msg.right_marker_detected_in_lastsec = true;
		}

		if (msg.ball_detected_in_lastsec && msg.left_marker_detected_in_lastsec
				&& msg.right_marker_detected_in_lastsec) {
			float goal_distance_half = (last_goal_msg.marker1_center_x
					- last_goal_msg.marker2_center_x) / 2.0;

			//olny x values
			//take center of goal = (M2 + M1)*0.5
			//offset = ball - center of goal
			//then offset scaled by half of the goal distance
			//ball_rel_goal = offset /( 0.5* (M2 - M1) )
			//final formula is =((2*B)-(M1+M2))/(M2-M1)
			msg.ball_rel_goal = ((2.0
					* last_ball_msg.ball_center_x)
					- (last_goal_msg.marker1_center_x
							+ last_goal_msg.marker2_center_x))
					/ (last_goal_msg.marker2_center_x
							- last_goal_msg.marker1_center_x);


			cout << "last ball center" << last_ball_msg << endl;
			cout << "last goal" << last_goal_msg << endl;
			cout << "our msg" << msg << endl;
		}
		//if ball seen, publish its relative position and distance
		if (msg.ball_detected_in_lastsec) {
			//take ball center
			//offset = image center - ball
			//scale offest from -1 for left image side to +1 for right image side
			//msg.ball_rel_image = (last_ball_msg.ball_center_x - (last_ball_msg.image_width*0.5)) / (last_ball_msg.image_width*0.5)
			//easier calculatoin version (but same as above)
			msg.ball_rel_image_x = (1.0 * last_ball_msg.ball_center_x
					/ (last_ball_msg.image_width / 2.0)) - 1.0;

			msg.ball_distance = last_ball_msg.ball_distance;

			msg.ball_rel_image_y = (1.0 * last_ball_msg.ball_center_y
								/ (last_ball_msg.image_height / 2.0)) - 1.0;

			//publish, where ball was detected
			msg.ball_in_top_or_bottom_camera = last_ball_msg_position;
		}
		/*
		float64 ball_rel_image_x
		float64 ball_rel_image_y
		uin8 ball_in_top_or_bottom_camera
		*/

		visionPublisher.publish(msg);
	}

};
//class
}//namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "vision_relativePosition");
	ros::NodeHandle nh;

	vision::Relative* relative = new vision::Relative(nh);

	ROS_INFO("spin now");
	ros::spin();

	return 0;
}

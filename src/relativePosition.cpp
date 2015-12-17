#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

//include custom messages
#include "RoNAOldo/goalPositionMsg.h"

namespace vision {
class Relative {
public:

	ros::NodeHandle _nh;

	ros::Publisher goalPublisher;


	Relative(ros::NodeHandle n)
	{
		_nh = n;


		//publish ballOffMiddle
		goalPublisher = _nh.advertise<RoNAOldo::goalPositionMsg>("goalPosition", 10);







	}

	~Relative()
	{


	}

}; //class
} //namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_relativePosition");
  ros::NodeHandle nh;



  ROS_INFO("spin now");
  ros::spin();


  return 0;
}

/************************/
/* HADES VIDEO STREAMER */
/************************/
/* todo:
 *	-add checks for max number of packets
 *	-clearer commenting
 *	-sensorCb
 */

#include "ros/ros.h"
#include "theora_image_transport/Packet.h"
#include <queue>
#include <stdio.h>

/***********************/
/* Global Declarations */
/***********************/

std::queue<theora_image_transport::Packet> cameraFrames;
std::vector<double> diffs;
ofstream output;

theora_image_transport::Packet camFrame;
double diff;
int size, count;

/* Handles camera/rgb/image_mono/theora topic */
void cameraCb(const theora_image_transport::Packet &frame){
    cameraFrames.push(frame); //ensures frames stored and can come out in order
}

/* Handles videoFeed/theora topic */
void videoFeedCb(const theora_image_transport::Packet &vidFrame){
	if (count == 20000){
		ROS_INFO("20000 packets counted");
		output.close();
		ros::shutdown();
	}
    camFrame = cameraFrames.front();
	cameraFrames.pop();
	if(camFrame.packetno == vidFrame.packetno){	//confirm frames in sync
		diff = vidFrame.header.stamp - camFrame.header.stamp;
		size = vidFrame.data.size() + 19;
		output << diff << ", " << size << "\n";
	}
	count++;
}

int main(int argc, char **argv){
	output.open("~/catkin_ws/src/hades_comm_handler/testing/timeTest.csv");
	count = 0;
	ros::init(argc, argv, "timeTester");	

	ros::NodeHandle nh;

	ros::Subscriber camSub = nh.subscribe("camera/rgb/image_mono/theora", 2000, cameraCb);
    ros::Subscriber vidSub = nh.subscribe("videoFeed/theora", 2000, videoFeedCb);
	
	ROS_DEBUG("initialised");

	ros::spin();
	return 0;
}
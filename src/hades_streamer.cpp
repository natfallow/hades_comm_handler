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
#include <hades_comm_handler/EZR_Pkt.h>

/***********************/
/* Global Declarations */
/***********************/

int totalSize;
uint8_t bytePos, EZRCount, byteModulo, granpos[8], packnum[8];

std::queue<uint8_t> dataStream;  //data field FIFO

hades_comm_handler::EZR_Pkt pkt; //ROS msg sent to teensy

ros::Publisher pub;

/************************/
/* Theora Packetisation */
/************************/
void theoraCb(const theora_image_transport::Packet &frame){
	ROS_DEBUG("frame received");

	/*Prepare packet parameters*/
		totalSize = frame.data.size() + 15;                //variable data field + 8B granulepos + 4B theoraFrameCount + 1B EZRCount + 1B byteModulo + 1B MsgType
		EZRCount = totalSize / 62 + (totalSize % 62 != 0); //number of EZR packets needed to send this message
		byteModulo = totalSize % 62;                       //number of data bytes in last packet
	/***************************/

	/*Read data field into FIFO*/
		for(int i = 0;i<=frame.data.size();i++)
			dataStream.push(frame.data[i]);
		ROS_DEBUG("datastream populated");
	/***************************/

	/*Convert int64 to byte arrays*/
		std::memcpy(granpos, &frame.granulepos, 8);    //assuming little endianness
		std::memcpy(packnum, &frame.packetno, 8);      //assuming little endianness
		ROS_DEBUG("gpos & pnum instantiated");
	/******************************/

	/*Construct first packet*/
		pkt.data[0] = 1;        //message type 1 for video
		pkt.data[1] = EZRCount; //number of EZR packets needed to send this message
		pkt.data[2] = byteModulo//number of data bytes in last packet

		bytePos = 3;
		while(bytePos<=6){ //next 4 bytes are theora.packetno field (int64 casts to uint8[4])
			pkt.data[bytePos] = packnum[bytePos-2];
			bytePos++;
		}

		while(bytePos<=14){	//next 8 bytes are theora.granulepos field (int64 cast to uint8[8])
			pkt.data[bytePos] = granpos[bytePos-6];
			bytePos++;
		}			

		while(bytePos<63 && !dataStream.empty()){ //populate remainder from data field
			pkt.data[bytePos] = dataStream.front();
			dataStream.pop();
			bytePos++;
		}
		
		pub.publish(pkt); //publish first packet
		ROS_INFO("packet 1");
	/************************/

	/*construct data packets*/
		for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){ //iterate through all data packets
			bytePos = 0;

			pkt.data[bytePos] = pktNo;	//first byte is packet position in msg
			
			while(bytePos<63 && !dataStream.empty()){ //populate remainder from data field
				pkt.data[bytePos] = dataStream.front();
				dataStream.pop();
				bytePos++;
			}

			pub.publish(pkt); //publish data packet
			ROS_DEBUG("packet %d",pktNo);
		} 
	/************************/
}


int main(int argc, char **argv){

	ros::init(argc, argv, "comms");	

	ros::NodeHandle nh;

	ros::Subscriber subVid = nh.subscribe("camera/rgb/image_mono/theora_throttle", 150, theoraCb);

	pub = nh.advertise<hades_comm_handler::EZR_Pkt>("serialOut", 512);
	
	ROS_DEBUG("initialised");

	ros::spin();
	return 0;
}

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
std::queue<hades_comm_handler::EZR_Pkt> pktQueue; //packet FIFO

hades_comm_handler::EZR_Pkt pkt; //ROS msg to be sent to teensy

ros::Publisher pub;

/************************/
/* Theora Packetisation */
/************************/

void Uint64toUint8Arr (uint8_t* buf, uint64_t var, uint32_t lowest_pos){
	buf[lowest_pos]     =   (var & 0x00000000000000FF) >> 0 ;
	buf[lowest_pos+1]   =   (var & 0x000000000000FF00) >> 8 ;
	buf[lowest_pos+2]   =   (var & 0x0000000000FF0000) >> 16 ;
	buf[lowest_pos+3]   =   (var & 0x00000000FF000000) >> 24 ;
	buf[lowest_pos+4]   =   (var & 0x000000FF00000000) >> 32 ;
	buf[lowest_pos+5]   =   (var & 0x0000FF0000000000) >> 40 ;
	buf[lowest_pos+6]   =   (var & 0x00FF000000000000) >> 48 ;
	buf[lowest_pos+7]   =   (var & 0xFF00000000000000) >> 56 ;
}

void theoraCb(const theora_image_transport::Packet &frame){
	ROS_DEBUG("frame received");

	/*Prepare packet parameters*/
		totalSize = frame.data.size() + 19;                //variable data field + 8B granulepos + 8B theoraFrameCount + 1B EZRCount + 1B byteModulo + 1B MsgType
		ROS_INFO("totalSize: %d",totalSize);
		EZRCount = totalSize / 62 + (totalSize % 62 != 0); //number of EZR packets needed to send this message
		byteModulo = totalSize % 62;                       //number of data bytes in last packet
	/***************************/

	/*Read data field into FIFO*/
		for(int i = 0;i < frame.data.size();i++)
			dataStream.push(frame.data[i]);
		ROS_DEBUG("datastream populated");
	/***************************/

	/*Convert int64 to byte arrays*/
		Uint64toUint8Arr(granpos, frame.granulepos, 0);    //assuming little endianness
		Uint64toUint8Arr(packnum, frame.packetno, 0);      //assuming little endianness
		ROS_DEBUG("gpos & pnum instantiated");
	/******************************/

	/*Construct first packet*/
		pkt.data[0] = 1;         //message type 1 for video
		pkt.data[1] = EZRCount;  //number of EZR packets needed to send this message
		pkt.data[2] = byteModulo;//number of data bytes in last packet

		bytePos = 3;
		while(bytePos<=10){ //next 4 bytes are theora.packetno field (int64 casts to uint8[4])
			pkt.data[bytePos] = packnum[bytePos-3];
			bytePos++;
		}

		while(bytePos<=18){	//next 8 bytes are theora.granulepos field (int64 cast to uint8[8])
			pkt.data[bytePos] = granpos[bytePos-11];
			bytePos++;
		}			

		while(bytePos<63 && !dataStream.empty()){ //populate remainder from data field
			pkt.data[bytePos] = dataStream.front();
			dataStream.pop();
			bytePos++;
		}
		while(bytePos<63){ //populate remainder with zeroes
			pkt.data[bytePos] = 0;
			bytePos++;
		}
		
		pktQueue.push(pkt); //publish first packet
		ROS_DEBUG("packet 1");
	/************************/

	/*construct data packets*/
		for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){ //iterate through all data packets
			pkt.data[0] = pktNo;	//first byte is packet position in msg
			bytePos = 1;

			while(bytePos<63 && !dataStream.empty()){ //populate remainder from data field
				pkt.data[bytePos] = dataStream.front();
				dataStream.pop();
				bytePos++;
			}
			while(bytePos<63){ //populate remainder with zeroes
				pkt.data[bytePos] = 0;
				bytePos++;
			}

			pktQueue.push(pkt); //publish data packet
			ROS_DEBUG("packet %d",pktNo);
		} 
	/************************/
}

/*Ensure packets come out evenly spaced*/
void timerCb(const ros::TimerEvent&){
	if(!pktQueue.empty()){
		pub.publish(pktQueue.front());
		pktQueue.pop();
		ROS_INFO("packet sent");
	}
}

int main(int argc, char **argv){
	//initialise ROS node
	ros::init(argc, argv, "comms");	
	
	//declare node handler
	ros::NodeHandle nh;

	//declare subscriber to theora video & publisher to serialOut
	ros::Subscriber subVid = nh.subscribe("camera/rgb/image_mono/theora_throttle", 150, theoraCb);

	pub = nh.advertise<hades_comm_handler::EZR_Pkt>("serialOut", 512);

	//declare timer to ensure packets spaced out
	ros::Timer pktTimer = nh.createTimer(ros::Duration(0.0006), timerCb);

	ROS_DEBUG("initialised");

	ros::spin();
	return 0;
}

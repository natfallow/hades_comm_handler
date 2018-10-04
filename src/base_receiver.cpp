/************************/
/* HADES VIDEO STREAMER */
/************************/
/* todo:
 * - 
 */

#include "ros/ros.h"
#include "theora_image_transport/Packet.h"
#include <queue>
#include <hades_comm_handler/EZR_Pkt.h>

/***********************/
/* Global Declarations */
/***********************/

uint8_t bytePos, EZRCount, byteModulo, granpos[8], packnum[4];

std::queue<uint8_t> dataField;  //data field FIFO

theora_image_transport::Packet frame; //reconstituted theora frame 

ros::Publisher pub;

/*************************/
/* Theora Reconstitution */
/*************************/

void theoraCb(const hades_comm_handler::EZR_Pkt &pkt){
	ROS_DEBUG("packet received")

	/*Handle first packet of frame*/
		if (pkt.data[0]==1){ //check if new frame
			ROS_DEBUG("New Frame");

			EZRCount = pkt.data[1]	//total packets in this frame

			bytePos = 2;
			while(bytePos<=5){ //next 4 bytes are theora.packetno field (int64 casts to uint8[4])
				packnum[bytePos-2] = pkt.data[bytePos];
				bytePos++;
			}
			
			while(bytePos<=13){	//next 8 bytes are theora.granulepos field (int64 cast to uint8[8])
				granpos[bytePos-6] = pkt.data[bytePos];
				bytePos++;
			}
			
			if(EZRCount == 1){ //if single packet frame, next byte is byte modulo
				byteModulo = pkt.data[bytePos];
				bytePos++;
			}

			while(bytePos<63){ //populate remainder from data field
				dataField.push(pkt.data[bytePos]);
				if(EZRCount == 1 && bytePos == byteModulo)
					break;
				bytePos++;
			}
		}
	/******************************/

	/*Handle subsequent packets*/
		for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){ //iterate through all data packets
			if (frame.data[0] != pktNo)
				ROS_DEBUG("mismatched packet position")
			
			bytePos = 1;
			if(pktNo == EZRCount){	//second byte of last packet is modulo
				byteModulo = pkt.data[bytePos]; 
				bytePos++;
			}

			while(bytePos<63){ //populate remainder from data field
				dataField.push(pkt.data[bytePos]);
				if(EZRCount == 1 && bytePos == byteModulo)
					break;
				bytePos++;
			}
			ROS_DEBUG("packet %d",pktNo);
		} 
	/***************************/

	/* prep & publish frame */
		frame.packetno = packnum;
		frame.granulepos = granpos;

		bytePos = 0;
		while(!dataField.empty()){
			frame.data[bytePos] = dataField.front();
			dataField.pop();
			bytePos++;
		}
		pub.publish(frame);
	/************************/
}

int main(int argc, char **argv){
	ros::init(argc, argv, "commsOut");	

	ros::NodeHandle re;

	ros::Subscriber subVid = re.subscribe("serialOut", 10000, serialCb);

	pub = re.advertise<theora_image_transport::Packet>("videoFeed", 150);
	
	ROS_DEBUG("initialised");

	ros::spin();
	return 0;
}

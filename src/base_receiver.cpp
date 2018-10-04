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

uint64_t Uint8ArrtoUint64 (uint8_t* var, uint32_t lowest_pos){
    return  (((uint64_t)var[lowest_pos+7]) << 56) | 
            (((uint64_t)var[lowest_pos+6]) << 48) |
            (((uint64_t)var[lowest_pos+5]) << 40) | 
            (((uint64_t)var[lowest_pos+4]) << 32) |
            (((uint64_t)var[lowest_pos+3]) << 24) | 
            (((uint64_t)var[lowest_pos+2]) << 16) |
            (((uint64_t)var[lowest_pos+1]) << 8)  | 
            (((uint64_t)var[lowest_pos])   << 0);
}

uint64_t Uint8ArrtoUint64_pnum (uint8_t* var, uint32_t lowest_pos){
    return  (((uint64_t)var[lowest_pos+3]) << 24) | 
            (((uint64_t)var[lowest_pos+2]) << 16) |
            (((uint64_t)var[lowest_pos+1]) << 8)  | 
            (((uint64_t)var[lowest_pos])   << 0);
}

/*************************/
/* Theora Reconstitution */
/*************************/

void serialCb(const hades_comm_handler::EZR_Pkt &pkt){
	ROS_INFO("packet received");

	/*Handle first packet of frame*/
		if (pkt.data[0]==1){ //check if new frame
			ROS_INFO("New Frame");
			while(!dataField.empty()){dataField.pop();}	//ensure data queue is empty 

			EZRCount = pkt.data[1];	//total packets in this frame
			ROS_INFO("EZRCount: %d",EZRCount);
			
			byteModulo = pkt.data[2];//data bytes in last packet of frame
			ROS_INFO("size: %d",((EZRCount-1)*62+byteModulo));
			
			bytePos = 3;

			while(bytePos<=6){ //next 4 bytes are theora.packetno field (int64 casts to uint8[4])
				packnum[bytePos-3] = pkt.data[bytePos];
				bytePos++;
			}
			frame.packetno = Uint8ArrtoUint64_pnum(packnum, 0);
			ROS_INFO("packetno");

			while(bytePos<=14){	//next 8 bytes are theora.granulepos field (int64 cast to uint8[8])
				granpos[bytePos-7] = pkt.data[bytePos];
				bytePos++;
			}
			frame.granulepos = Uint8ArrtoUint64(granpos, 0); 
			ROS_INFO("granulepos");

			while(bytePos<63){ //populate remainder from data field
				dataField.push(pkt.data[bytePos]);
				if(EZRCount == 1 && bytePos == byteModulo)
					break;
				bytePos++;
			}
		ROS_INFO("First packet handled");
		}
		
	/******************************/

	/*Handle subsequent packets*/
		if (pkt.data[0] <= EZRCount){ //iterate through all data packets
			ROS_INFO("handling packet %d",pkt.data[0]);

			for(bytePos = 1; bytePos < 63; bytePos++){ //populate remainder from data field
				if(pkt.data[0]==EZRCount && bytePos == byteModulo)
					break;
				dataField.push(pkt.data[bytePos]);
			}
		} 
	/***************************/

	/* Prepare and send frame */
		if(pkt.data[0] == EZRCount){
			ROS_INFO("last packet of frame");
			bytePos = 0;
			while(!dataField.empty()){
				frame.data.push_back(dataField.front());
				ROS_INFO("%d data read",bytePos);
				dataField.pop();
				bytePos++;
			}
			pub.publish(frame);
		}
	/**************************/
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

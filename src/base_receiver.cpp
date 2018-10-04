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

uint64_t Uint8ArrtoUint64B (uint8_t* var, uint32_t lowest_pos){
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

			EZRCount = pkt.data[1];	//total packets in this frame
			ROS_INFO("%d",EZRCount);

			bytePos = 2;
			while(bytePos<=5){ //next 4 bytes are theora.packetno field (int64 casts to uint8[4])
				packnum[bytePos-2] = pkt.data[bytePos];
				bytePos++;
			}
			ROS_INFO("packetno read");
			while(bytePos<=13){	//next 8 bytes are theora.granulepos field (int64 cast to uint8[8])
				granpos[bytePos-6] = pkt.data[bytePos];
				bytePos++;
			}
			ROS_INFO("granulepos read");
			if(EZRCount == 1){ //if single packet frame, next byte is byte modulo
				ROS_INFO("single packet");
				byteModulo = pkt.data[bytePos];
				bytePos++;
			}

			while(bytePos<63){ //populate remainder from data field
				ROS_INFO("%d",bytePos);
				dataField.push(pkt.data[bytePos]);
				if(EZRCount == 1 && bytePos == byteModulo)
					break;
				bytePos++;
			}
		}
		ROS_INFO("First packet handled");
	/******************************/

	/*Handle subsequent packets*/
		for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){ //iterate through all data packets
			ROS_INFO("%d packets handled",pktNo);
			
			ROS_INFO("packet number checked");
			bytePos = 1;
			if(pktNo == EZRCount){	//second byte of last packet is modulo
				ROS_INFO("last packet");
				byteModulo = pkt.data[bytePos]; 
				bytePos++;
			}

			while(bytePos<63){ //populate remainder from data field
				dataField.push(pkt.data[bytePos]);
				if(EZRCount == 1 && bytePos == byteModulo)
					break;
				bytePos++;
			}
			ROS_INFO("packet %d",pktNo);
		} 
ROS_INFO("left for");
	/***************************/

	/* prep & publish frame */
		frame.granulepos = Uint8ArrtoUint64(granpos, 0);    //assuming little endianness
		frame.packetno = Uint8ArrtoUint64B(packnum, 0);
		ROS_INFO("set gpos and pnum");
		bytePos = 0;
		while(!dataField.empty()){
			ROS_INFO("%d data read",bytePos);
			frame.data[bytePos+1] = dataField.front();
			dataField.pop();
			bytePos++;
		}
		ROS_INFO("set data");
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

#include "ros/ros.h"
#include "theora_image_transport/Packet.h"
#include <queue>
#include <hades_comm_handler/EZR_Pkt.h>

/***********************/
/* Global Declarations */
/***********************/

int totalSize, bytePos;
uint8_t EZRCount, byteModulo, granpos[8], packnum[8];

ros::Publisher pub;

hades_comm_handler::EZR_Pkt pkt;

/************************/
/* Theora Packetisation */
/************************/

void theoraCb(const theora_image_transport::Packet &frame){
  ROS_INFO("theora callback");
  totalSize = frame.data.size() + 15;                    //variable data field + 8B granulepos + 4B theoraFrameCount + 1B EZRCount + 1B byteModulo + 1B MsgType
  EZRCount = totalSize / 62 + (totalSize % 62 != 0); //number of EZR packets needed to send this message
  byteModulo = totalSize % 62;                       //number of data bytes in last packet
  
  std::queue<uint8_t> dataStream;   //theora data field

  for(int i = 0;i<=frame.data.size();i++)
    dataStream.push(frame.data[i]);
  ROS_INFO("datastream populated");
  
  std::memcpy(granpos, &frame.granulepos, 8);    //assuming little endianness
  std::memcpy(packnum, &frame.packetno, 8);      //assuming little endianness
  ROS_INFO("gpos & pnum instantiated");

  /*Construct first packet*/

  pkt.data[0] = 1;        //message type 1 for video
  pkt.data[1] = EZRCount; //number of EZR packets needed to send this message
  bytePos = 2;
  while(bytePos<=5){
    pkt.data[bytePos] = packnum[bytePos-2];
    bytePos++;
  }
  while(bytePos<=13){
    pkt.data[bytePos] = granpos[bytePos-6];
    bytePos++;
  }
  if(EZRCount == 1){
    pkt.data[bytePos] = byteModulo;
    bytePos++;
  }
  while(bytePos<63 && !dataStream.empty()){
    pkt.data[bytePos] = dataStream.front();
    ROS_INFO("%d",pkt.data[bytePos]);
    dataStream.pop();
    bytePos++;
  }
  pub.publish(pkt);
  ROS_INFO("first packet");

  /*construct data packets*/
  for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){
    bytePos = 0;
    pkt.data[bytePos] = pktNo;      //first byte is packet position in msg
    
    if(pktNo == EZRCount){
      pkt.data[bytePos] = byteModulo; //second byte of last packet is modulo
      bytePos++;
    }
    
    while(bytePos<63 && !dataStream.empty()){      //loop to fill data
      pkt.data[bytePos] = dataStream.front();
      dataStream.pop();
      bytePos++;
    }
    pub.publish(pkt); //publish packet to teensy
    ROS_INFO("%d packet",pktNo);
  } 
}


int main(int argc, char **argv){

  ros::init(argc, argv, "comms");

  ros::NodeHandle nh;

  ros::Subscriber subVid = nh.subscribe("camera/rgb/image_mono/theora_throttle", 150, theoraCb);

  pub = nh.advertise<hades_comm_handler::EZR_Pkt>("serialOut", 512);
    ROS_INFO("begin");


  ros::spin();

  return 0;
}

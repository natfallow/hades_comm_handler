#include "ros/ros.h"
#include "theora_image_transport/Packet.h"
#include "std_msgs/String.h"
#include <queue>
#include "EZR_Pkt.h"

/*Theora Packetisation*/
void theoraCb(const theora_image_transport::Packet::ConstPtr &frame){
  int totalSize = frame.data.size() + 15;                    //variable data field + 8B granulepos + 4B theoraFrameCount + 1B EZRCount + 1B byteModulo + 1B MsgType
  uint8_t EZRCount = totalSize / 62 + (totalSize % 62 != 0); //number of EZR packets needed to send this message
  uint8_t byteModulo = totalSize % 62;                       //number of data bytes in last packet
  
  std::queue<uint8_t> dataStream;   //theora data field
  for(int i = 0;i<=frame.data.size();i++)
    dataStream.push(frame.data[i]);
  
  int64_t granpos[8];               //theora granulepos field
  int64_t packnum[8];               //theora packetno field
  
  std::memcpy(granpos, &frame.granulepos, 8);    //assuming little endianness
  std::memcpy(packnum, &frame.packetno, 8);      //assuming little endianness

  /*Construct first packet*/
  EZR_Pkt pkt;
  pkt.data[0] = 1;        //message type 1 for video
  pkt.data[1] = EZRCount; //number of EZR packets needed to send this message
  int i = 2;
  while(i<=5){
    pkt.data[i] = packnum[i-2];
    i++;
  }
  while(i<=13){
    pkt.data[i] = granpos[i-6];
    i++;
  }
  if(EZRCount == 1){
    pkt.data[i] = byteModulo;
    i++;
  }
  while(i<=63 && !dataStream.empty()){
    pkt.data[i] = dataStream.front();
    dataStream.pop();
    i++;
  }

  /*construct data packets*/
  for (uint8_t pktNo = 2; pktNo <= EZRCount; pktNo++){
    EZR_Pkt pkt;    //generate packet to send over EZR connection
    
    int i = 0;
    pkt.data[i] = pktNo;      //first byte is packet position in msg
    
    if(pktNo == EZRCount){
      pkt.data[i] = byteModulo; //second byte of last packet is modulo
      i++;
    }
    
    while(i <= 63 && !dataStream.empty()){      //loop to fill data
      pkt.data[i] = dataStream.front();
      dataStream.pop();
      i++;
    }
    pub.publish(pkt); //publish packet to teensy
  }

  /*last packet includes byte modulo*/
  EZR_Pkt pkt;
  pkt.data
}

void sensorCb(const sensor.msg &msg){
  //probably just stick the sensor stuff in a FIFO to wait to be sent out
}

int main(int argc, char **argv){
  ros::init(argc, argv, "comms");

  ros::NodeHandle nh;

  ros::Subscriber subSen = nh.subscribe("sensors", 150, sensorCb);
  ros::Subscriber subVid = nh.subscribe("camera/rgbd/mono/theora_throttle", 150, theoraCb);

  ros::Publisher pub = nh.advertise<EZR_Pkt>("serialOut", 512);

  ros::spin();

  return 0;
}
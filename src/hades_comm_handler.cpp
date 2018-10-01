#include "ros/ros.h"
#include "theora_image_transport/Packet.h"
#include "std_msgs/String.h"
#include <vector>
#include "EZR_Pkt.h"

void theoraCb(const theora_image_transport::Packet::ConstPtr &frame){
  int totalSize = frame.data.size() + 15;                    //variable data field + 8B granulepos + 4B theoraFrameCount + 1B EZRCount + 1B byteModulo + 1B MsgType
  uint8_t EZRCount = totalSize / 62 + (totalSize % 62 != 0); // number of EZR packets needed to send this message
  uint8_t byteModulo = totalSize % 62;                       //number of data bytes in last packet
  
  std::vector<uint8_t> dataStream;

  //some loop or something to put data into dataStream FIFO; order is EZRCount,(int32)frame.packetno,frame.granulepos,frame.data

  for (uint8_t i = 1; i < EZRCount + 1; i++){
    EZR_Pkt pkt;    //generate packet to send over EZR connection

    pkt.data[1] = i;  //packet number
    if (i == EZRCount)
      pkt.data[2] = byteModulo; //if last packet, second byte should be number of remaining data bytes
    //pkt.data[] rest pulled off datastream;

    pub.publish(pkt); //send packet to teensy via rosserial
  }
}

void sensorCb(const sensor.msg &msg){
  //probably just stick the sensor stuff in a FIFO to wait to be sent out
}

int main(int argc, char **argv){
  ros::init(argc, argv, "comms");

  ros::NodeHandle nh;

  ros::Subscriber subSen = nh.subscribe("sensors", 150, sensorCb);
  ros::Subscriber subVid = nh.subscribe("camera/rgbd/mono/theora", 150, theoraCb);

  ros::Publisher pub = nh.advertise<EZR_Pkt>("serialOut", 512);

  ros::spin();

  return 0;
}
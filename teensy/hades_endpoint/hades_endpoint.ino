/*TODO:
  Currently only for theora packets; need to generalise for video / sensor / control packet types*/

#define TEENSY3_6 //controller definition 

/*Pin Definitions*/
/*#define debugPin        13*/
#define slaveSelectPin  10
#define interruptOut    24
#define interruptIn     25

#include <ros.h>
#include <cppQueue.h>
#include <SPI.h>
#include <std_msgs/String.h>
#include </home/hades/sketchbook/libraries/ros_lib/hades_comm_handler/EZR_Pkt.h>


/*ROS serial setup*/
uint8_t rcvdFromRos[63]; //received packet to be put in buffer

Queue txBuf(63, 512, FIFO); //transmit buffer (63 byte records, 512 records total, FIFO mode)
Queue rxBuf(1, 1024, FIFO); //receive buffer (1 byte records, 1024 records total, FIFO mode)

ros::NodeHandle  nh;        //Declare ROS node handler

hades_comm_handler::EZR_Pkt rcvd_pkt; //declare received packet

/*SPI var setup*/
uint8_t SPICounter;
bool recieveFlag = false;

SPISettings settings(2450000, MSBFIRST, SPI_MODE1);


uint8_t rByte;
/***********/

/*When packet received from serial*/
void messageCb(const hades_comm_handler::EZR_Pkt& pkt) {
  txBuf.push(&pkt.data);
}

/*declare subscriber to theora_image_transport.packet.msgs
  on the camera/rgb/image_mono/theora topic,
  execute messageCb when message received*/
ros::Subscriber<hades_comm_handler::EZR_Pkt> sub("serialOut", &messageCb);

/*declare publisher rcvd that publishes rcvd_frame on the received topic*/
ros::Publisher rcvd("serialTest", &rcvd_pkt);




/*when packet received from radio*/
void SPIRecieve() {
  SPI.beginTransaction(settings);

  digitalWrite(slaveSelectPin, LOW);

  rByte = SPI.transfer(0);                //Intialising every SPI could be slowing system

  //rxBuf.push(SPI.transfer(0));                   //TEST

  digitalWrite(slaveSelectPin, HIGH);

  SPI.endTransaction();
  
  rxBuf.push(&rByte);
  Serial.println(rByte);
  SPICounter++;
}

/*packet sent to radio*/
void SPISend() {
  //digitalWrite(debugPin, HIGH - digitalRead(debugPin)); //flash LED when packet received
  
  uint8_t txPacket[63];
  txBuf.pop(&txPacket);
  
  SPI.beginTransaction(settings);
  digitalWrite(slaveSelectPin, LOW); //This is making the interrupt not work...
  

  for (int i = 0; i <= 62; i++) {
    SPI.transfer(txPacket[i]);
        
    //digitalWrite(debugPin, HIGH - digitalRead(debugPin)); //flash LED when packet received
    delayMicroseconds(9);
  }
  digitalWrite(slaveSelectPin, HIGH);
  SPI.endTransaction();
}

void setup()
{
  SPI.begin();

  SPICounter = 0;

  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);

  pinMode(interruptOut, OUTPUT);
  digitalWrite(interruptOut, LOW);

  pinMode(interruptIn, INPUT);
  attachInterrupt(interruptIn, SPIRecieve, RISING);

  //pinMode(debugPin, INPUT);

  nh.getHardware()->setBaud(2000000);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rcvd);
}

void loop()
{
  if (!txBuf.isEmpty()) {
    SPISend();
    delayMicroseconds(6000);
  }
  
  if (SPICounter == 63) {
    for (int i = 0; i < 63; i++) {
      rxBuf.pop(&rcvd_pkt.data[i]);      
    }
    rcvd.publish(&rcvd_pkt);
    SPICounter = 0;
  }
  nh.spinOnce();
}



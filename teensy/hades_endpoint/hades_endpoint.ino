#include <EEPROM.h>

/*TODO:
  Currently only for theora packets; need to generalise for video / sensor / control packet types*/

#define TEENSY3_6 //controller definition 

/*Pin Definitions*/
/*#define debugPin        13*/
#define slaveSelectPin  10
#define interruptOut    24
#define interruptIn     25

#include <ros.h>
#include <std_msgs/String.h>
#include </home/hades/sketchbook/libraries/ros_lib/hades_comm_handler/EZR_Pkt.h>

#include <cppQueue.h>
#include <SPI.h>
#include <TimerOne.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/*ROS serial setup*/
uint8_t rcvdFromRos[63]; //received packet to be put in buffer

Queue txBuf(63, 512, FIFO); //transmit buffer (63 byte records, 512 records total, FIFO mode)

ros::NodeHandle  nh;        //Declare ROS node handler

hades_comm_handler::EZR_Pkt rcvd_pkt; //declare received packet

/*SPI var setup*/
uint8_t SPICounter;
bool recieveFlag = false;

SPISettings settings(2450000, MSBFIRST, SPI_MODE1);


int packetCount = 0;

int byteError = 0;

uint8_t rByte[63];
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




/*when byte received from radio*/
void SPIRecieve() {
  SPI.beginTransaction(settings);

  digitalWrite(slaveSelectPin, LOW);

  rByte[SPICounter] = SPI.transfer(0);                //Intialising every SPI could be slowing system
  //SPICounter--;
  //rByte[SPICounter] = SPI.transfer(0);                //Intialising every SPI could be slowing system
  
  digitalWrite(slaveSelectPin, HIGH);

  SPI.endTransaction();

  SPICounter++;
}

/*packet sent to radio*/
void SPISend() {
  uint8_t txPacket[63];
  txBuf.pop(&txPacket);

  SPI.beginTransaction(settings);
  

  for (int i = 0; i <= 62; i++) {
    digitalWrite(slaveSelectPin, LOW);

    delayMicroseconds(9);
    
    SPI.transfer(txPacket[i]);

    delayMicroseconds(9);

    digitalWrite(slaveSelectPin, HIGH);
  }
  
  SPI.endTransaction();
}

//void timeOut(){
//SPICounter = 0;
//}

void setup()
{
  //Timer1.initialize(1000000);

  SPI.begin();

  SPICounter = 0;

  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);

  pinMode(interruptOut, OUTPUT);
  digitalWrite(interruptOut, LOW);

  pinMode(interruptIn, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptIn), SPIRecieve, RISING);

  //Timer1.attachInterrupt(timeOut);


  nh.getHardware()->setBaud(2000000);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rcvd);

  //Timer1.start();

}

void loop()
{
  if (!txBuf.isEmpty()) {
    SPISend();
    //delayMicroseconds(1900);
    //delayMicroseconds(300);
  }

  if (SPICounter == 63) {
    packetCount++;
    memcpy(rcvd_pkt.data, rByte, 63);
    

    rcvd.publish(&rcvd_pkt);
    
    SPICounter = 0;
  }
  nh.spinOnce();
}

//

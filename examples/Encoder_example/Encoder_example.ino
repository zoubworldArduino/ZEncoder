//EXAMPLE OF MOTOR WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS24 if COM24
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS24 & 

//        rostopic list
//        rostopic echo /1/counter 

//        rostopic echo -p /1/counter


// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Captor


#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>

#if defined(BOARD_ID_Pilo)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
#include "ros.h"
ros::NodeHandle  nh;

#elif defined(BOARD_ID_Captor)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM0.serial2)
#define ROS_BAUDRATE 57600

#include "ros.h"
ros::NodeHandle  nh;

#else
//#include <Servo.h> 
#include "ros.h"
ros::NodeHandle  nh;
#endif

#include <ZCmdMotor.h>

#define MySerial P_COM0.serial2
#define PcomSerial MySerial



#define M1_CA P_Encoder[1-1].Pin.IA  
#define M1_CB P_Encoder[1-1].Pin.IB   
#define M1_MP -1/*rouge-rouge*/
#define M1_MM  -1 /*blanc-blanc*/
/*
#define M2_CA  P_Encoder[2-1].Pin.IA  
#define M2_CB  P_Encoder[2-1].Pin.IB  
#define M2_MP  -1 
#define M2_MM  -1 

*/
#define M2_CA  P_Encoder[4-1].Pin.IA  
#define M2_CB  P_Encoder[4-1].Pin.IB  
#define M2_MP  -1 /*rouge*/
#define M2_MM  -1 /*blanc*/


#define M3_CA  P_Encoder[3-1].Pin.IA  
#define M3_CB  P_Encoder[3-1].Pin.IB  
#define M3_MP  -1 /*rouge*/
#define M3_MM  -1 /*blanc*/







//ZEncoder enc(A0,A2,FULL, NULL);
CMDMOTOR cmd1(M1_CA, M1_CB, M1_MP, M1_MM);
CMDMOTOR cmd2(M2_CA, M2_CB, M2_MP, M2_MM);
CMDMOTOR cmd3(M3_CA, M3_CB, M3_MP, M3_MM);

int count=0;
void privateIntHandler1() {
  cmd1.getEncoder()->update();
  count++;
}
void privateIntHandler2() {
  cmd2.getEncoder()->update();
}
void privateIntHandler3() {
  cmd3.getEncoder()->update();
}

// the setup function runs once when you press reset or power the board
void setupCMD() {
  MySerial.print("setup CMD \r\n");
  delay(500);
  cmd1.setPin(M1_CA, M1_CB, M1_MP, M1_MM);
  cmd2.setPin(M2_CA, M2_CB, M2_MP, M2_MM);
  cmd3.setPin(M3_CA, M3_CB, M3_MP, M3_MM);
  cmd1.setup();
  cmd2.setup();
  cmd3.setup();
  
  MySerial.print("setup CMD end \r\n");
  delay(500);
  cmd1.getEncoder()->attachEncoderInt(privateIntHandler1);
  cmd2.getEncoder()->attachEncoderInt(privateIntHandler2);
cmd3.getEncoder()->attachEncoderInt(privateIntHandler3);

cmd1.setup( &nh,	"/1/pwm","/1/speed");
cmd2.setup(&nh,	"/2/pwm","/2/speed");
cmd3.setup(&nh,	"/3/pwm","/3/speed");
 cmd1.getEncoder()->setup( &nh,	"/1/counter");
 cmd2.getEncoder()->setup( &nh,	"/2/counter");
 cmd3.getEncoder()->setup( &nh,	"/3/counter");
 
}
void displayEncoder() {

  PcomSerial.print("displayCMD1()  \r");
  PcomSerial.print("CMD1 getValue=\t");
  PcomSerial.print(cmd1.getEncoder()->getValue());
  PcomSerial.print("\t, getDirection=");
  PcomSerial.print(cmd1.getEncoder()->getDirection());
  PcomSerial.print("\t, getSpeed= ");
  PcomSerial.print(cmd1.getEncoder()->getSpeed());
  PcomSerial.print("\t, getDeltaValue=");
  PcomSerial.print(cmd1.getEncoder()->getDeltaValue());
  PcomSerial.print("\r\n");

  PcomSerial.print("CMD2 getValue=\t");
  PcomSerial.print(cmd2.getEncoder()->getValue());
  PcomSerial.print("\t, getDirection=");
  PcomSerial.print(cmd2.getEncoder()->getDirection());
  PcomSerial.print("\t, getSpeed= ");
  PcomSerial.print(cmd2.getEncoder()->getSpeed());
  PcomSerial.print("\t, getDeltaValue=");
  PcomSerial.print(cmd2.getEncoder()->getDeltaValue());
  PcomSerial.print("\r\n");

}










void setup()
{/*
MySerial.begin(115200);//9600
MySerial.println("Setup");
   */

 nh.initNode(); 

   setupCMD();
/*
   MySerial.println("Setup End");
*/



   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
      delay(10);
    }
    
}

void loop()
{
 cmd1.loop();
  cmd2.loop();
  cmd3.loop();
  
/*
MySerial.println("loop");


 



delay(500);
    /// command
//PID/ENCODER/
//for(int i=0;i<1000;i++)
{
  
  displayEncoder();
 MySerial.println("count");MySerial.println(i);
   delay(200);
  
  }
 //   delay(1000);//wait the flush serial on the line  before re boost
//   jumpInBoot();// I finish, I return on boot for next sketch. to run me again reset.  
 */   
    nh.loginfo("loop()");
   nh.spinOnce();
    delay(200);
}





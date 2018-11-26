/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _PZEncoder_H
#define _PZEncoder_H

#include <Arduino.h>

#ifdef ROS_USED 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#endif 

/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/
// Another piece of state for rotation state machine
enum Rotation {
  CLOCKWISE = 2, COUNTERCLOCKWISE = 3, UNCERTAIN = 4
};

enum eMode {
  QUARTER = 0, FULL = 1
};
#define ENABLE_SPEED 1

typedef void (*ZEncodervoidFuncPtr)(void);
class ZEncoder {

public:
  //  ZEncoder(int pinA,int pinB, void (*optionalCallBack)(int));
  ZEncoder(int pinA, int pinB, eMode mymode, void (*optionalCallBack)(int));
  signed int getValue();
  void setValue(int newValue);
  void resetValue();
  signed int getDeltaValue();
  void update();
  int getDirection();
  void attachEncoderInt(ZEncodervoidFuncPtr userFunc);
  
#if ENABLE_SPEED

  /** 
   * Return the speed of the encoder.
   *
   * Calculate the speed (steps per second) for the encoder.
   * The sampling period is set using the setPeriod() method.
   * If the encoder is used to enter numbers or scan through menus, the speed 
   * can be used to accelerate the display (eg, skip larger values for each click).
   *
   * \return The speed in clicks per second.
   */
  signed int getSpeed(void);
  /* reset speed value to zero
  because speed is based on the last tick event when no tick happen the speed isn't updated normaly,
  here we add a computation based on last event tha time past since this event for low speed motion,
  this introduce an history from the past and consider a continuity, as in some case it isn't truth, we add this reset function
  example when you reset you PID or stop it at restart the data can be wrong.
  */
 void resetSpeed(void);
#endif
  volatile int value;
  
  void simulate(signed int value);
  
  
  void setSerialDebug(HardwareSerial * SerialDebug);
  
#ifdef ROS_USED 
    void setup( ros::NodeHandle  *myNodeHandle,	const char   *	topic);
	void loop();
#endif 
  private:
#ifdef ROS_USED 
    ros::NodeHandle  *nh;
    std_msgs::Int16 counter_msg;//speed //deltaD//D
    ros::Publisher *pub_counter;
#endif

  HardwareSerial * SerialDebug;

  volatile int ValueOld;

  // Encoder value
  uint8_t state;     // latest state for the encoder

  void (*optCallBack)(int);
  signed char pinIC1;
  signed char pinIC2;
  eMode mode;
#if ENABLE_SPEED    
  // Velocity data
  //  uint16_t  _period;  // velocity calculation period
  //  uint16_t  _count;   // running count of encoder clicks
  signed int _spd;     // last calculated speed (no sign) in clicks/second
  unsigned long _timeLast;  // last time read
#endif
  void privateIntHandler();  //not used
};
#endif
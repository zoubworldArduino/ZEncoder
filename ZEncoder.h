/** @file ZEncoder.h
An library that manage encoder like PDEC peripheral but in software with pin interrupt.


*/
#ifndef _PZEncoder_H
#define _PZEncoder_H

#include <Arduino.h>
#include <assert.h>

#ifdef ROS_USED 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
   
#endif 
//#define OPTIMIZE 1
#define ENABLE_SPEED 1
/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/

//! Another piece of state for rotation state machine
enum Rotation {
  CLOCKWISE = 2, COUNTERCLOCKWISE = 3, UNCERTAIN = 4
};

//! Mode accuracy.
enum eMode {
  QUARTER = 0 //!< define a tick at each edge of waveform of IA ant IB pin, so 4 tick per rotation, it use 2 interruptions.
  , FULL = 1 //!< define a tick per 4 edges of waveform of IA ant IB pin, so 1 tick per rotation, it use 1 interruption.
};
//#define ENABLE_SPEED 1

typedef void (*ZEncodervoidFuncPtr)(void);
class ZEncoder {

public:
  //  ZEncoder(int pinA,int pinB, void (*optionalCallBack)(int));
  /** constructor 
  **/
  ZEncoder(int pinA// pin IA that read the encoder output A, this pin must support interupt
  , int pinB// pin IA that read the encoder output B, this pin should support interupt
  , eMode mymode//! the mode of accuracy
  , void (*optionalCallBack)(int)// the call back function where you will call instance.update();
  );
  /** get teh absolute value
  @return the absolute value of encoder
  */
  signed int getValue();
  /** set the absolute value  
  */
  void setValue(int newValue//!< new value
  );
  /** set the absolute value to 0  
  */
  void resetValue();
  /** get the delta of value since last call to this function.
  @return the absolute value of encoder
  */
  signed int getDeltaValue();
  /** perform the processing on the interruption, it should be link to the call back.
  */
  void update();
  /** get the current direction
  */
  int getDirection();
  /** atttach manualy an interupt call back
  @deprecated
  */
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
  /** reset speed value to zero
  because speed is based on the last tick event when no tick happen the speed isn't updated normaly,
  here we add a computation based on last event tha time past since this event for low speed motion,
  this introduce an history from the past and consider a continuity, as in some case it isn't truth, we add this reset function
  example when you reset you PID or stop it at restart the data can be wrong.
  */
 void resetSpeed(void);
#endif
  volatile int value;
  /**
  @deprecated
  */
  void simulate(signed int value);
  
  /** setup a debug channel to have output on serial
  the drawback it that it waste cpu cycle, and you an lost some tick
  but usefull for debug.
  */
  void setSerialDebug(HardwareSerial * SerialDebug);
  
//@{
	/** setup the refresh rate of the topic speed
	*/
  void setRefreshRateUs(uint32_t intervalTime //!< duration between 2 topic in Micro Seconde
  );  
#ifdef ROS_USED 
   /** @name ROS IPA
*/

  /** the ros initialisation	
	*/
    void setup( ros::NodeHandle  *myNodeHandle//!< the ROS node handler
	,	const char   *	topic//!< the topic for position displayed in ROS
	);
	/** the ros initialisation	
	*/
     void setup( ros::NodeHandle * myNodeHandle//!< the ROS node handler
	 ,	const char   *	topic//!< the topic for position displayed in ROS
	 , const char   *	topicspeed//!< the topic for speed displayed in ROS
	 );
	/** function to be called in your main loop.
	*/
	void loop();
//@}
#endif
 
  private:
    uint32_t rate;
    
#ifdef OPTIMIZE
      int * addIC1;
      int * addIC2;
      unsigned int maskIC1;
      unsigned int maskIC2;
#endif
      
#ifdef ROS_USED 
    ros::NodeHandle  *nh;
    std_msgs::Int16 counter_msg;//speed //deltaD//D
    ros::Publisher *pub_counter;
    unsigned long timestamp;
    
    
     #if ENABLE_SPEED
    
    std_msgs::Int32 speed_msg;
    ros::Publisher *pub_speed;    
    
  #endif
  
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
  //signed int _spd;     // last calculated speed (no sign) in clicks/second
  //unsigned long _timeLast;  // last time read
  
  signed char _inc;
  signed int _spd;     // last calculated speed (no sign) in clicks/second
  unsigned long _spd_previous_time;  // previous to last time read
  unsigned long _spd_time;  // last time read  
  unsigned long _spd_previous_time_avg;// last time read  for average
  volatile int _spd_ValueOld;// old value for average
 


#endif
  void privateIntHandler();  //not used
};
#endif

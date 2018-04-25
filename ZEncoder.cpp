#include <ZEncoder.h>



/** give direction since last getDeltaValue()*/
int ZEncoder::getDirection() {
  return ((value - ValueOld) == 0) ?
      UNCERTAIN :
      (((value - ValueOld) > 0) ? CLOCKWISE : COUNTERCLOCKWISE);
}

// Gets the value on the encoder.  It's an integer, positive or negative.
signed int ZEncoder::getValue() {
  noInterrupts();
  int value2 = value;
  interrupts();
  return value2;
}
#if ENABLE_SPEED
signed int ZEncoder::getSpeed(void) {

  noInterrupts();
  signed int speed = _spd;
  unsigned long timeLast = _timeLast;
  unsigned long timen = micros();
  interrupts();

  signed int s = (speed > 0) ? 1 : -1;  // one tips with direction
  if ((signed int)(timen - timeLast) >= s * 1000000 / speed) // if slower than before compute as slow methode( without tips) 
      {

    speed = s * (1000000 / (timen - timeLast));

  }

  return (speed);
}
#endif

// Gets the in/de-crement of value on the encoder since last read.  It's an integer, positive or negative.

signed int ZEncoder::getDeltaValue() {
  noInterrupts();
  int t = value - ValueOld;
  ValueOld = value;
  interrupts();
  return t;
}

void ZEncoder::resetValue() {
  setValue(0);
}

void ZEncoder::setValue(int newValue) {
  noInterrupts();
  value = newValue;
  ValueOld = value;
  interrupts();
}

/**
 * This will enumerate a GroveEncoder on a particular pin.
 * You can provide an optional callback, or poll the "getValue()" API.
 * mode equal to QUARTER count 4 for a cycle(1 per phase), equal to full it count 1 per cycle(less accurate, but support high frequency).
 */
ZEncoder::ZEncoder(int pinA, int pinB, eMode mymode,
    void (*optionalCallBack)(int)) {

#if ENABLE_SPEED
  _timeLast = micros();
#endif
  pinIC1 = pinA;
  pinIC2 = pinB;
SerialDebug=0;
  if (SerialDebug) 
  {
    SerialDebug->print("ZEncoder(");
    SerialDebug->print((signed int) pinIC1);
    SerialDebug->print(",");
    SerialDebug->print((signed int) pinIC2);
    SerialDebug->print(") : ");
  }
  // Initialize values
  setValue(0);
  mode = mymode;
  optCallBack = optionalCallBack;

  // Initialize pins
#ifdef INPUT_PULLUP
  pinMode(pinIC1, INPUT_PULLUP);
  pinMode(pinIC2, INPUT_PULLUP);
#else
  pinMode(pinIC1, INPUT);
  digitalWrite(pinIC1, HIGH);
  pinMode(pinIC2, INPUT);
  digitalWrite(pinIC2, HIGH);
#endif
// Set up interrupts
//attachEncoderInt(privateIntHandler);
}

void ZEncoder::setSerialDebug(Uart * mySerialDebug)
{
  SerialDebug=mySerialDebug;
}

/* simulate a captor, do not use if the captor is connected ! !
*/
void ZEncoder::simulate(signed int value) 
{
  pinMode(pinIC1, OUTPUT);
  pinMode(pinIC2, OUTPUT);
  digitalWrite(pinIC1, HIGH);
  digitalWrite(pinIC2, HIGH);
  for(int i=0; i<value; )
  {
  digitalWrite(pinIC1, LOW);i++;
  digitalWrite(pinIC2, LOW);i++;
  digitalWrite(pinIC1, HIGH);i++;
  digitalWrite(pinIC2, HIGH);i++;
  }
    for(int i=0; i>value; )
  {
  digitalWrite(pinIC2, LOW);i--;
  digitalWrite(pinIC1, LOW);i--;
  digitalWrite(pinIC2, HIGH);i--;
  digitalWrite(pinIC1, HIGH);i--;
  }

  pinMode(pinIC1, INPUT_PULLUP);
  pinMode(pinIC2, INPUT_PULLUP);
}
void ZEncoder::attachEncoderInt(ZEncodervoidFuncPtr userFunc) {
if (SerialDebug) 
{
  SerialDebug->print("attachEncoderInt(");
  SerialDebug->print((signed int) pinIC1);
  SerialDebug->print(":");
  SerialDebug->print((signed int) digitalPinToInterrupt(pinIC1));
  SerialDebug->print(",");
  SerialDebug->print((signed int) pinIC2);
  SerialDebug->print(":");
  SerialDebug->print((signed int) digitalPinToInterrupt(pinIC2));
  SerialDebug->print(") ; ");
  SerialDebug->print((unsigned int) &userFunc);
}
if (mode == QUARTER) // count one tip per phase(4 phases per cycle)
      {
        if (SerialDebug) 
    SerialDebug->print(" QUARTER mode ");
    if (digitalPinToInterrupt(pinIC1) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(pinIC1), userFunc, CHANGE);
#ifdef PCINT_VERSION
    else
    if(digitalPinToPCINT(pinIC1)!=NOT_AN_INTERRUPT)
    attachPCINT(digitalPinToPCINT(pinIC1), userFunc,CHANGE); // attach a PinChange Interrupt to our pin on the rising edge
#endif
    else{
      if (SerialDebug) 
      SerialDebug->print("ZEncoder::error can't attach interrupt to pin1");
    }
    if (digitalPinToInterrupt(pinIC2) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(pinIC2), userFunc, CHANGE);
#ifdef PCINT_VERSION
    else
    if(digitalPinToPCINT(pinIC2)!=NOT_AN_INTERRUPT)
    attachPCINT(digitalPinToPCINT(pinIC2), userFunc,CHANGE); // attach a PinChange Interrupt to our pin on the rising edge
#endif
    else
    {
      if (SerialDebug) 
        SerialDebug->print("ZEncoder::error can't attach interrupt to pin2");}
  } else // count 1 tip by cycle of 4 phases.
  { if (SerialDebug) 
    SerialDebug->print(" FULL mode");
    if (digitalPinToInterrupt(pinIC1) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(pinIC1), userFunc, RISING);
#ifdef PCINT_VERSION
    else
    if(digitalPinToPCINT(pinIC1)!=NOT_AN_INTERRUPT)
    attachPCINT(digitalPinToPCINT(pinIC1), userFunc,RISING); // attach a PinChange Interrupt to our pin on the rising edge
#endif
    else
    { if (SerialDebug) 
      SerialDebug->print("ZEncoder::error can't attach interrupt to pin1");
    }
     if (SerialDebug) 
    SerialDebug->print("\r\n ");
  }

// (RISING, FALLING and CHANGE all work with this library)

}
/*
 // This function only exists due to the singleton issue.
 void ZEncoder::privateIntHandler()
 {
 update();
 
// if(optCallBack != NULL)
// optCallBack(singleton->getValue());
 }*/

//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2
//QUARTER  --->        0   1  2   3  4    5   6   7   8   9
//QUARTER  <---       -8  -7 -6  -5  -4  -3   -2 -1   0   1
//FULL     ---->    0..     1               2
//FULL    <----                   -2             -1   ...0        
//        new      new      old      old
//s       pin2     pin1     pin2     pin1     Result
//---     ----     ----     ----     ----     ------
//0       0        0        0        0        no movement
//1       0        0        0        1        +1
//2       0        0        1        0        -1
//3       0        0        1        1        +2  (assume pin1 edges only)
//4       0        1        0        0        -1
//5       0        1        0        1        no movement
//6       0        1        1        0        -2  (assume pin1 edges only)
//7       0        1        1        1        +1
//8       1        0        0        0        +1
//9       1        0        0        1        -2  (assume pin1 edges only)
//10      1        0        1        0        no movement
//11      1        0        1        1        -1
//12      1        1        0        0        +2  (assume pin1 edges only)
//13      1        1        0        1        -1
//14      1        1        1        0        +1
//15      1        1        1        1        no movement
// Simple, easy-to-read "documentation" version 
//
void ZEncoder::update(void) {
#if ENABLE_SPEED
  unsigned long _time = micros();
#endif
  signed char inc = 0;
  if (mode == QUARTER) {
    uint8_t s = state & 3;
    if (digitalRead(pinIC1))
      s |= 4;
    if (digitalRead(pinIC2))
      s |= 8;

    switch (s) {
    case 0:
    case 5:
    case 10:
    case 15:
      break;
    case 1:
    case 7:
    case 8:
    case 14:
      inc = +1;
      break;
    case 2:
    case 4:
    case 11:
    case 13:
      inc = -1;
      break;
    case 3:
    case 12:
      inc = 2;
      break;
    default:
      inc = 2;
      break;
    }

    state = (s >> 2);
  } else {
    if (digitalRead(pinIC2) == 1)
      inc = 1;
    else
      inc = -1;
  }
  value += inc;
  
#if ENABLE_SPEED
  // handle the encoder velocity calc

  if ((_time - _timeLast) > 0) {
    _spd = inc * (1000000 / (_time - _timeLast));
  }
  _timeLast = _time;

  /*
   if ((_time - _timeLast) >= _period)
   {
   _spd = _count * (1000/_period);
   _timeLast = millis();
   _count = 0;
   }*/
#endif
}
/*==================================================================================================*/
/*==================================================================================================*/
/*==================================================================================================*/














































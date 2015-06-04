// Espresso RC control code
#include <Servo.h>

const int throttleRCPin = 2; // RC input pin 'elevator', used for forward/reverse
const int steeringRCPin = 3; // RC input pin, 'aileron', used for steering
const int LifterRCPin = 4; // RC input pin, 'throttle', used for lifter
const int LeftESCPin = 10; // ESC output pin to LHS motors
const int RightESCPin = 11; // ESC output pin to RHS motors
const int LifterESCPin = 17; // ESC output pin to lifter motors
const int LifterPotPin = 15; // Analog input pin to determin lifter position

long LastTriggered;
float Timeout = 5000;
long EndTime;
long LastAuto = 0;
int steeringSignal = 1500, throttleSignal = 1500, LifterSignal = 1500;
int LifterPosition = 0; // value read from the pot

Servo LeftESC;
Servo RightESC;
Servo LifterESC;

void setup() {
  LeftESC.attach(LeftESCPin);
  RightESC.attach(RightESCPin);
  LifterESC.attach(LifterESCPin);
  Serial.begin(9600);
  LastTriggered = millis();
  pinMode(throttleRCPin, INPUT);
  pinMode(steeringRCPin, INPUT);
  pinMode(LifterRCPin, INPUT);
}

void loop() {
    RCMode();
}

void RCMode() {
  Serial.print("RCMode v0.8");
  throttleSignal = pulseIn(throttleRCPin, HIGH, 25000); // Read the pulse width of
  steeringSignal = pulseIn(steeringRCPin, HIGH, 25000); // each channel
  LifterSignal = pulseIn(LifterRCPin, HIGH, 25000);
  if (throttleSignal < 800) throttleSignal = 1500;
  if (steeringSignal < 800) steeringSignal = 1500;
  if (LifterSignal < 800) LifterSignal = 1500;
  Serial.print(", throttle input: ");
  Serial.print(throttleSignal);
  Serial.print(", steering input: ");
  Serial.print(steeringSignal);
  Serial.print(", Left output: ");
  Serial.print(constrain((-throttleSignal + steeringSignal) / 2 + 1500, 1000, 2000));
  LeftESC.write(constrain((-throttleSignal + steeringSignal) / 2 + 1500, 1000, 2000));
  Serial.print(", Right output: ");
  Serial.print(constrain((throttleSignal + steeringSignal) / 2, 1000, 2000));
  RightESC.write(constrain((throttleSignal + steeringSignal) / 2, 1000, 2000));

  LifterPosition = analogRead(LifterPotPin);
  Serial.print(", lifter command: ");
  Serial.print(LifterSignal);
  if (LifterPosition > 900) {
    LifterSignal = max(1500, LifterSignal);
  }
  if (LifterPosition < 100) {
    LifterSignal = min(1500, LifterSignal);
  }
  LifterESC.write(constrain(LifterSignal, 1400, 1600));
  Serial.print(", current position: ");
  Serial.print(LifterPosition);
  Serial.print(", output: ");
  Serial.println(LifterSignal);
}

// // MultiChannels
// //
// // rcarduino.blogspot.com
// //
// // A simple approach for reading three RC Channels using pin change interrupts
// //
// // See related posts -
// // http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// // http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// // http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
// //
// // rcarduino.blogspot.com
// //

// // include the pinchangeint library - see the links in the related topics section above for details
// #include <PinChangeInt.h>

// #include <Servo.h>

// // Assign your channel in pins
// #define THROTTLE_IN_PIN 8
// #define STEERING_IN_PIN 9
// #define AUX_IN_PIN 10

// // Assign your channel out pins
// #define THROTTLE_OUT_PIN 5
// #define STEERING_OUT_PIN 6
// #define AUX_OUT_PIN 7

// // Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// // We will use the objects to output the signals we read in
// // this example code provides a straight pass through of the signal with no custom processing
// Servo servoThrottle;
// Servo servoSteering;
// Servo servoAux;

// // These bit flags are set in bUpdateFlagsShared to indicate which
// // channels have new signals
// #define THROTTLE_FLAG 1
// #define STEERING_FLAG 2
// #define AUX_FLAG 4

// // holds the update flags defined above
// volatile uint8_t bUpdateFlagsShared;

// // shared variables are updated by the ISR and read by loop.
// // In loop we immediatley take local copies so that the ISR can keep ownership of the
// // shared ones. To access these in loop
// // we first turn interrupts off with noInterrupts
// // we take a copy to use in loop and the turn interrupts back on
// // as quickly as possible, this ensures that we are always able to receive new signals
// volatile uint16_t unThrottleInShared;
// volatile uint16_t unSteeringInShared;
// volatile uint16_t unAuxInShared;

// // These are used to record the rising edge of a pulse in the calcInput functions
// // They do not need to be volatile as they are only used in the ISR. If we wanted
// // to refer to these in loop and the ISR then they would need to be declared volatile
// uint32_t ulThrottleStart;
// uint32_t ulSteeringStart;
// uint32_t ulAuxStart;

// void setup()
// {
//   Serial.begin(9600);

//   Serial.println("multiChannels");

//   // attach servo objects, these will generate the correct
//   // pulses for driving Electronic speed controllers, servos or other devices
//   // designed to interface directly with RC Receivers
//   servoThrottle.attach(THROTTLE_OUT_PIN);
//   servoSteering.attach(STEERING_OUT_PIN);
//   servoAux.attach(AUX_OUT_PIN);

//   // using the PinChangeInt library, attach the interrupts
//   // used to read the channels
//   PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
//   PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
//   PCintPort::attachInterrupt(AUX_IN_PIN, calcAux,CHANGE);
// }

// void loop()
// {
//   // create local variables to hold a local copies of the channel inputs
//   // these are declared static so that thier values will be retained
//   // between calls to loop.
//   static uint16_t unThrottleIn;
//   static uint16_t unSteeringIn;
//   static uint16_t unAuxIn;
//   // local copy of update flags
//   static uint8_t bUpdateFlags;

//   // check shared update flags to see if any channels have a new signal
//   if(bUpdateFlagsShared)
//   {
//     noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

//     // take a local copy of which channels were updated in case we need to use this in the rest of loop
//     bUpdateFlags = bUpdateFlagsShared;

//     // in the current code, the shared values are always populated
//     // so we could copy them without testing the flags
//     // however in the future this could change, so lets
//     // only copy when the flags tell us we can.

//     if(bUpdateFlags & THROTTLE_FLAG)
//     {
//       unThrottleIn = unThrottleInShared;
//     }

//     if(bUpdateFlags & STEERING_FLAG)
//     {
//       unSteeringIn = unSteeringInShared;
//     }

//     if(bUpdateFlags & AUX_FLAG)
//     {
//       unAuxIn = unAuxInShared;
//     }

//     // clear shared copy of updated flags as we have already taken the updates
//     // we still have a local copy if we need to use it in bUpdateFlags
//     bUpdateFlagsShared = 0;

//     interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
//     // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
//     // service routines own these and could update them at any time. During the update, the
//     // shared copies may contain junk. Luckily we have our local copies to work with :-)
//   }

//   // do any processing from here onwards
//   // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
//   // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
//   // the interrupt routines and should not be used in loop

//   // the following code provides simple pass through
//   // this is a good initial test, the Arduino will pass through
//   // receiver input as if the Arduino is not there.
//   // This should be used to confirm the circuit and power
//   // before attempting any custom processing in a project.

//   // we are checking to see if the channel value has changed, this is indicated
//   // by the flags. For the simple pass through we don't really need this check,
//   // but for a more complex project where a new signal requires significant processing
//   // this allows us to only calculate new values when we have new inputs, rather than
//   // on every cycle.
//   if(bUpdateFlags & THROTTLE_FLAG)
//   {
//     if(servoThrottle.readMicroseconds() != unThrottleIn)
//     {
//       servoThrottle.writeMicroseconds(unThrottleIn);
//     }
//   }

//   if(bUpdateFlags & STEERING_FLAG)
//   {
//     if(servoSteering.readMicroseconds() != unSteeringIn)
//     {
//       servoSteering.writeMicroseconds(unSteeringIn);
//     }
//   }

//   if(bUpdateFlags & AUX_FLAG)
//   {
//     if(servoAux.readMicroseconds() != unAuxIn)
//     {
//       servoAux.writeMicroseconds(unAuxIn);
//     }
//   }

//   bUpdateFlags = 0;
// }


// // simple interrupt service routine
// void calcThrottle()
// {
//   // if the pin is high, its a rising edge of the signal pulse, so lets record its value
//   if(digitalRead(THROTTLE_IN_PIN) == HIGH)
//   {
//     ulThrottleStart = micros();
//   }
//   else
//   {
//     // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
//     // this gives use the time between the rising and falling edges i.e. the pulse duration.
//     unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
//     // use set the throttle flag to indicate that a new throttle signal has been received
//     bUpdateFlagsShared |= THROTTLE_FLAG;
//   }
// }

// void calcSteering()
// {
//   if(digitalRead(STEERING_IN_PIN) == HIGH)
//   {
//     ulSteeringStart = micros();
//   }
//   else
//   {
//     unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
//     bUpdateFlagsShared |= STEERING_FLAG;
//   }
// }

// void calcAux()
// {
//   if(digitalRead(AUX_IN_PIN) == HIGH)
//   {
//     ulAuxStart = micros();
//   }
//   else
//   {
//     unAuxInShared = (uint16_t)(micros() - ulAuxStart);
//     bUpdateFlagsShared |= AUX_FLAG;
//   }
// }










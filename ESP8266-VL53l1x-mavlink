/* This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the robot. The speed of the wheels is
  controled by the PWM signal using the analogWrite() function.
  
  Author: Udom
  Date: 9 Oct 2019
  *** you may need to disconnect the motor control pins before uploading the sketch ***
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#define stepAng  3       // step angle
#define numStep   60      // = 180/stepAng 
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms
#define TurnDelay 300       // turn for 300ms
#define MinDistance 100     // 100mm
#define MaxDistance 1000     // 100mm
#define scannerPin D0
Servo scanner;     
VL53L1X sensor;

int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value


volatile unsigned long next;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"

int FOV = 120; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
int res = 3;
uint16_t distances[72];

int target = 0;

char serial_buffer[15];


//=======================================================================
// software timer interrupt: counts clock cycles at 80MHz
void inline motorCtrl_ISR(void){



  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
}

void setup() {
  Serial.begin(1500000);
  scanner.attach(scannerPin);  //attach scanner servo
  Wire.begin(D5, D6); 
  Wire.setClock(400000); // use 400 kHz I2C

  //Initialize the timer
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(motorCtrl_ISR);
  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
  interrupts();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);  
   Serial.println(" setup ");
}







void loop() { 

  
  pos += dir;
  scanner.write(pos*stepAng);
    

///////////////////////////////////////////////////////////////////////

lidarAngle = map(pos, 0, 180, -90, 90);      
messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);

 if(lidarAngle%res == 0){ // get a distance reading for each res (3 degree) step

  val = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }


//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = val-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 5; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 4000; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -FOV/2;
  uint8_t frame = 12;
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);




      
  if (pos == numStep)
  {
    dir = -1;
  }
  else if (pos == 0)
  {
    dir = 1;
  }

 
}
}

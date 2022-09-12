// Includes
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <Servo.h>
#include <Wire.h>
#include <VL53L1X.h>

#define POTI_PIN A0
#define FCbaud 1500000

int pothigh = 700;
int potlow = 300;

int res = 2;
int FOV = 120;

int potiValue = 0; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
uint16_t distances[72];
int target = 0;
unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;

HardwareSerial Serial1(USART1);

VL53L1X sensor;
int val;              // LiDAR measured value

class Sweeper
{
 Servo servo;              // the servo
 int pos;              // current servo position 
 int increment;        // increment to move for each interval
 int  updateInterval;      // interval between updates
 unsigned long lastUpdate; // last update of position

public: 
 Sweeper(int interval)
 {
   updateInterval = interval;
   increment = 1;
 }
 
 void Attach(int pin)
 {
   servo.attach(pin);
 }
 void Detach()
 {
   servo.detach();
 }
 void Update()
 {
   if((millis() - lastUpdate) > updateInterval)  // time to update
   {
     lastUpdate = millis();
     pos += increment;
     servo.write(pos);
     if ((pos >= 180) || (pos <= 0)) // end of sweep
     {
       // reverse direction
       increment = -increment;
     }
   }
 }
};


char serial_buffer[15];
Sweeper sweeper1(20);

void setup() {/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial1.begin(FCbaud); // USB
  sweeper1.Attach(A1);
  Wire.begin(PB6, PB7); //SDA, 
  Wire.setClock(400000); // use 400 kHz I2C

 }
//======================================


// Scanning fuction. Adapt to your needs
int16_t Dist = 0;    // Distance to object in centimeters


void loop() {///////////////////////////////////////////////////////////////////////////////////////////////////////////////

potiValue = analogRead(POTI_PIN);
lidarAngle = map(potiValue, potlow, pothigh, -60, 60);          
messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);

if(lidarAngle%res <=target)
  {
moveservo();
send_pos();
read_lidar();
  
  }

}

void moveservo(){

sweeper1.Update();
}
 
void read_lidar(){
 
  val = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Dist = val/10;
      }
    
  



void send_pos(){///////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 196;    
  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = Dist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 30; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 500; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -FOV/2;
  uint8_t frame = 12;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message
  
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
  
  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1,196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
  

}


#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

VL53L1X sensor;
Servo myservo;

unsigned long previousMillis = 0;
const long interval = 100;

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
char serial_buffer[15];

int ledState = LOW;

int pos = 500;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value
#define stepAng           1      // step angle
#define numStep           1000        // = 180/stepAng

void setup()
{

  myservo.attach(29);  // attaches the servo on pin 9 to the servo object

  Serial2.begin(1500000); // FC
  Serial.begin(500000);
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  pinMode(16, OUTPUT);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(5000);
  sensor.startContinuous(5);


  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
}
int16_t Dist = 0;    // Distance to object in centimeters
void loop()
{
  sensor.read();
  command_servo();
  command_lidar();
  command_mavlink();
 // command_led();
  command_print();

}


void command_servo() {
 pos += dir;
myservo.writeMicroseconds(pos);
 if (pos == 2100)
  {
    dir = -50;
  }
  
  else if (pos == 900)
  {
    dir = 50;
  }

}

void command_lidar() {

  Dist = (sensor.ranging_data.range_mm / 10);
  messageAngle = map(pos, 500, 2500, 0, 72);

}


void command_mavlink() {


  int sysid = 1;
  //< The component sending the message.
  int compid = 196;
  uint64_t time_usec = 0;
  uint8_t sensor_type = 0;
  distances[messageAngle] = Dist - 2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 2;
  uint16_t min_distance = 10;
  uint16_t max_distance = 650;
  float increment_f = 0;
  float angle_offset = -10;
  uint8_t frame = 12;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message

  mavlink_msg_obstacle_distance_pack(sysid, compid, &msg, time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    mavlink_msg_heartbeat_pack(1, 196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }

}

void command_led() {

  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(16, ledState);
}

void command_print() {
  sensor.read();

  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("angle: ");
  Serial.print(pos*stepAng);
  Serial.println();
}

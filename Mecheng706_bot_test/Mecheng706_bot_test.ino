/* To control the Rover, copy and paste the code below into the Arduino
 software*/

#include <Servo.h>
#include "main.h"

const byte left_front = 46;
const byte left_rear = 49;
const byte right_rear = 47;
const byte right_front = 48;

Servo left_front_motor;  // create servo object to control a servo
Servo left_rear_motor;  // create servo object to control a servo
Servo right_rear_motor;  // create servo object to control a servo
Servo right_front_motor;  // create servo object to control a servo

int speed_val = 100;
int speed_change;

int range_diff;
//bool go_fw = true;
int sensor_dist;
float actual_dist_lr;
float actual_dist_sr;


void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  //Open serial comms to bluetooth module
}

void loop(void) //main loopM
{
  // Variable declarations
  int* side_dist;

  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:
      machine_state =  running();
      break;
    case STOPPED:
      machine_state =  stopped();
      break;
  }

   go_forward_and_align();
   //side_dist = get_side_distances();
   //Serial.println(*(side_dist));
   //Serial.println(*(side_dist+1));
   //Serial.println("");

   //align();
/*
    while (analogRead(A3)>300){
      cw();
      delay(750);
      while ((analogRead(A2)-analogRead(A1)) > 50) {
     cw();//Reverse for normal use
    }
    }
    */

/*
  // Convert long range sensor reading into distance from the wall
  sensor_dist = analogRead(A1);
  //sensor_dist = (analogRead(A2)+analogRead(A1))/2;
  actual_dist_lr = 317756*(pow(sensor_dist,(-1.172)));

  sensor_dist = analogRead(A3);
  actual_dist_sr = 107698*(pow(sensor_dist,(-1.15)));
*/
}

/*----------------------------------------------------------------*/

void go_forward_and_align()
{
  // Variable declarations
  int* side_dist;
  side_dist = get_side_distances();
  //Speed_adj is gain needed to turn motor back on course.
  int speed_adj=2*((*(side_dist+1)) - (*(side_dist )));


  left_front_motor.writeMicroseconds(1500 + speed_val + speed_adj);
  left_rear_motor.writeMicroseconds(1500 + speed_val + speed_adj);
  right_rear_motor.writeMicroseconds(1500 - speed_val + speed_adj);
  right_front_motor.writeMicroseconds(1500 - speed_val + speed_adj);
}

int* get_side_distances() {
  static int dist[2];

  dist[0] = 317756*(pow(analogRead(A1),(-1.172)));
  dist[1] = 317756*(pow(analogRead(A2),(-1.172)));
  return dist;
}

void align(void)
{
  // Variable declarations
  int* side_dist;
  side_dist = get_side_distances();


  // Front side sensor reading is greater than rear side sensor. Thus turn ccw.
  while (( *(side_dist) - *(side_dist+1)) > 15) {
    ccw();
    // Update side sensor readings.
    side_dist = get_side_distances();
  }
  stop();

   // Front side sensor reading is less than rear side sensor. Thus turn cw.
  while (( *(side_dist) - *(side_dist+1)) < 0) {
    cw();
    // Update side sensor readings.
    side_dist = get_side_distances();
  }
  stop();
}

/*----------------------------------------------------------------*/

STATE initialising() {
  //Do some initialising
  Serial.println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  Serial.println("Enabling Motors...");
  enable_motors();
  return RUNNING;
}

STATE running() {
  read_serial_command();
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  if (is_battery_voltage__not_OK()) return STOPPED;
  return RUNNING;
}

STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;

  disable_motors();
  slow_flash_LED_builtin();


  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    Serial.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
    Serial.println("Please Re-charge Lipo");
  }
  //500ms timed if statement to check lipo and output speed settings
  if (!is_battery_voltage__not_OK()) {
    counter_lipo_voltage_ok++;
    if (counter_lipo_voltage_ok > 20) { //Making sure lipo voltage is stable
      counter_lipo_voltage_ok = 0;
      return RUNNING;
    }
  } else counter_lipo_voltage_ok = 0;
  return STOPPED;
}

/* To control the Rover, copy and paste the code below into the Arduino
 software*/

#include <Servo.h>
#include "main.h"

const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 48;
const byte right_front = 49;

Servo left_font_motor;  // create servo object to control a servo
Servo left_rear_motor;  // create servo object to control a servo
Servo right_rear_motor;  // create servo object to control a servo
Servo right_font_motor;  // create servo object to control a servo

int speed_val = 100;
int speed_change;
//bool go_fw = true;


void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  //Open serial comms to bluetooth module
}

void loop(void) //main loopM
{
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
  };


while ((analogRead(A1)-analogRead(A2)) >50) {
  ccw();
}
while ((analogRead(A1)-analogRead(A2)) <-50) {
  cw();
}
stop();

}


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

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

boolean is_battery_voltage__not_OK()
{
  static unsigned long previous_millis;
  
  if (millis() - previous_millis > 500) { //500ms timed if statement to check lipo and output speed settings
    previous_millis = millis();
    Serial.print("Lipo level:");
    int Lipo_level_cal;
    //the voltage of a LiPo cell depends on its chemistry and varies from about 2.7-3.1 V (discharged) = 620(3.1V Min)
    //to about 4.20-4.25 V (fully charged) = 820(4.1V Max)
    Lipo_level_cal = (analogRead(A0) - 620);
    Lipo_level_cal = Lipo_level_cal * 100;
    Lipo_level_cal = Lipo_level_cal / 200;
    speed_val += speed_change;
    if (speed_val > 1000)
      speed_val = 1000;
    speed_change = 0;
    Serial.print(Lipo_level_cal);
    Serial.println("%");
    if (Lipo_level_cal < 0) return false;
  }
  return false;
}

void range_and_speed_settings()
{
  static unsigned long previous_millis;
 
  if (millis() - previous_millis > 500) {  //500ms timed if statement to check lipo and output speed settings
    previous_millis = millis();
    Serial.print("Range1:");
    Serial.println(analogRead(A1));
    Serial.print("Range2:");
    Serial.println(analogRead(A2));
  }

}

void read_serial_command()
{
  if (Serial.available()) {
    char val = Serial.read();
    Serial.print("Speed:");
    Serial.print(speed_val);
    Serial.print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        Serial.println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        Serial.println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        Serial.println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        Serial.println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        Serial.println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        Serial.println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        Serial.println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        Serial.println("+");
        break;
      default:
        stop();
        Serial.println("stop");
        break;
    }

  }

}

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to the servo object
  left_rear_motor.detach();  // detach the servo on pin left_rear to the servo object
  right_rear_motor.detach();  // detach the servo on pin right_rear to the servo object
  right_font_motor.detach();  // detach the servo on pin right_front to the servo object

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to the servo object
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to the servo object
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to the servo object
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to the servo object
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}





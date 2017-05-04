
// Controls the servo and gets reads the sonar 
#include <Servo.h>
#include <NewPing.h>

/*
 * angle 98 is then sodar is straight ahead
 */

 const byte servo_pin = 5;
 const byte sodar_trig = 2;
 const byte sodar_echo = 3;
 const int max_distance = 200;      //set max distance in cm
 const int max_angle = 138;     //the maximum angle the servo can turn to
 const int min_angle = 58;
 const int increment = 1;     //the amount the motor increments whilst scanning the area(in degrees)
 const int delay_between_increments = 200; //delay between pings on micro seconds
 
 int last_increment = 0; //used to ensure servo only increments every 30ms
 boolean turning_direction = true;     //gives the direction the servo is turning. true = angle is increasing
 Servo servo_centre;      //create servo object to control servo motor to rotat the sonar
 NewPing sodar(sodar_trig, sodar_echo, max_distance);

/*----------------------------------------------------------------*/
// Servo Code: initilises at angle 93, angle 98 is straight ahead for the sodar
//             range of 81, for depth array, angle 58=0, angle 138 = 80
/*----------------------------------------------------------------*/
void enable_servo ()
 {
   servo_centre.attach(servo_pin);  // attaches the servo on pin left_front to the servo object
   //Serial1.println("Enabling Servo...");
 }

 void disable_servo ()
{
  servo_centre.detach();  // detach the servo on pin left_front to the servo object
  
  pinMode(servo_pin, INPUT);
}

void turn_servo_to(int degree) {     //turns the servo TO the given degrees, 
 servo_centre.write(degree); 
}

void turn_servo_by(int degree) {    //turns the servo BY the given angle
  int angle = servo_centre.read();
  servo_centre.write(angle + degree);
}

void increment_servo() {    //increments the servo , jumping to max once min angle has been reached

  int angle = servo_centre.read();
  if((angle - increment) < min_angle) {
    servo_centre.write(max_angle);
  } else if(angle > max_angle) {
    servo_centre.write(max_angle);
  } else {
    servo_centre.write((angle-increment));
  }
        
  
}

void increment_servo_in_range(int max_range, int min_range) {  //increments the servo within set range, jumping to max when min  angle has been reached
 
  if(max_range > max_angle) max_range = max_angle;    //ensuring that set range is within servo limit
  if(min_range < min_angle) min_range = min_angle;
  int angle = servo_centre.read();
  
   if((angle - increment) < min_range) {
    servo_centre.write(max_range);
  } else if(angle > max_range) {
    servo_centre.write(max_range);
  } else {
    servo_centre.write((angle-increment));
  }
}

int servo_angle() {   //returns the value of the servos angle in degrees
  return servo_centre.read();
}


/*----------------------------------------------------------------*/
// Sodar CODE:
/*----------------------------------------------------------------*/
 int sodar_ping(){      //returns the distance of the ping in cm
  int dist;
  dist = sodar.ping_cm();
  return dist;
 }

int ave_sodar_ping(int number_of_pings) {   //returns the average distance ofer set number of pings
  int t = sodar.ping_median(number_of_pings);
  int dist = sodar.convert_cm(t);
  return dist;
}

/*----------------------------------------------------------------*/
// location of object/wall CODE:  only ones that should be called from other files
/*----------------------------------------------------------------*/
boolean can_ping() {    //checks if enoguh time has passed for acurate pinging. Should be rn before any other servo/sodar code
  if (((int)millis() - last_increment) < delay_between_increments) return false;
  last_increment = (int) millis();
  return true;  
}

int* ping_location() {      //returns angle then distance of a ping to be used for mapping.
   static int location[2];
   
   location[0] = servo_angle();
   location[1] = sodar_ping()*10;  

   return location;
}

int* sweep() {
  increment_servo();
  return ping_location();  
}

int* sweep_in_range(int max_range, int min_range) {   //sweeps within given range, returngs ping values if in range, null if out of range
  increment_servo_in_range(max_range, min_range);
  if (servo_angle() > min_range && servo_angle() < max_range) {
    return 0;
  }
  return ping_location();
}

int depth_array_index(int angle) {     //returns the index for the depth array size at the current angle
  int i = angle - 58;
  return i;
}





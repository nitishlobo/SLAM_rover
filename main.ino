/* Main file to control robot navigation using state machine

Note: Finish all TODO's before merging into master.
*/
#include <Servo.h>
#include "state_definitions.h"

#define ITERATION_SIZE 5
// Set maximum distance (mm) at which obstacle is detected.
#define OB_LIMIT 255
#define ROBOT_WIDTH 210

// Ultrasonic sensor
static int sonar[ITERATION_SIZE];

// Pin numbers on Arduino Mega that the servo wheels are attached to.
const byte left_front = 46;
const byte left_rear = 49;
const byte right_rear = 47;
const byte right_front = 48;

// Servo objects from the <Servo.h> library.
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;

// Calibration gain for IR sensors
float ir_calibration = 0.2;

// Direction 0 is the initial direction in which the robot is placed.
int direction = 0;
int total_turn = 0;

// Collect distances from left and right IR sensors and middle sonar sensor.
int d_front_left, d_front_middle, d_front_right;
// 10mm allowance for IR sensors to be out of dead zone.
int d_wall = ROBOT_WIDTH + 10;

// Make robot continue to go forward in the OBSTACLE state rather than immediately switch to END_OBSTACLE_STRAFE.
int ob_strafe_time, ob_forward_time;
bool ob_in_front = false;
bool ob_on_right = false;
bool strafed_past_ob = false;

int ob_counter = 0;
int forward_counter = 0;



// Set servo motors to a small initial angle of 200 microseconds
// Ie. 72 degrees since 1000 ms is 360 degrees.
int speed = 200;
// Dynamic speed change of servos depending on input from serial command.
int speed_change;


bool first_loop = false;

//Set a flag to do testing in testing state
bool test_enable=false;



static float posChanges[3];


void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial1.begin(115200);  //Open Serial1 comms to bluetooth module

  //Serial1.println("Time HD FL SNR FR SF SB STATE");
  //Starts up I2C bus for reading from MPU
 MPU_setup();
}

void loop(void) //main loopM
{
  static STATE machine_state = INITIALISING;
  if (machine_state != INITIALISING ) {
    update_and_output();
  }
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      //Serial1.println("1");
      break;
    case TEST:
      machine_state=test();
      Serial1.println("2");
      break;
    case RUNNING:
      machine_state =  running();
      Serial1.println("3");
      break;
    case STARTUP:
      machine_state =  startup();
      Serial1.println("4");
      break;
    case WALL_FOLLOW:
      machine_state =  wall_follow();
      Serial1.println("5");
      break;
    case CORNER:
    Serial1.println("6");
      machine_state =  corner();
      direction++;
      total_turn++;
      if(total_turn>11){
        machine_state=stopped();
      }
      if (direction > 3) {
        // Increment the turn distance from the wall by robot width
        d_wall = d_wall + ROBOT_WIDTH;
        // Direction resets to 0 after 4 turns because robot is now travelling in the same direction as initial direction.
        direction = 0;
        //Has finished the first loop
        ir_calibration=0.1;
        first_loop=false;
      }

      break;
    case BEGIN_OBSTACLE_STRAFE:
      machine_state = begin_obstacle_strafe();
      Serial1.println("7");
      break;
    case OBSTACLE:
      machine_state =  obstacle();
      Serial1.println("8");
      break;
    case END_OBSTACLE_STRAFE:
      machine_state =  end_obstacle_strafe();
      Serial1.println("9");
      break;
    case STOPPED:
      machine_state =  stopped();
      Serial1.println("10");
      break;
    case BACK:
      machine_state = back();
      Serial1.println("12");
  }



  /*----------------------------------------*/
}

void update_and_output(){
  // Variable declarations
  int* side_dist;
  int* front_ir_dist;
  int* sonar_d;
  float heading;
  //Distance away from the wall
  side_dist = get_side_distances();
  front_ir_dist = get_front_ir_distances();
  d_front_left = *(front_ir_dist);
  d_front_right = *(front_ir_dist+1);
  d_front_middle = get_sonar_distance();
  heading = Read_MPU();

  Serial1.print(millis());
  Serial1.print(" ");
  Serial1.print(heading);
  Serial1.print(" ");
  Serial1.print(d_front_left);
  Serial1.print(" ");
  Serial1.print(d_front_middle);
  Serial1.print(" ");
  Serial1.print(d_front_right);
  Serial1.print(" ");
  Serial1.print(*side_dist);
  Serial1.print(" ");
  Serial1.print(*(side_dist+1));
  Serial1.print(" ");
}

void update_no_output(){
  // Variable declarations
  int* side_dist;
  int* front_ir_dist;
  int* sonar_d;
  float heading;
  //Distance away from the wall
  side_dist = get_side_distances();
  front_ir_dist = get_front_ir_distances();
  d_front_left = *(front_ir_dist);
  d_front_right = *(front_ir_dist+1);
  d_front_middle = get_sonar_distance();
  heading = Read_MPU();

}

/*----------------------------------------------------------------*/
// SPIRAL CODE:
/*----------------------------------------------------------------*/

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

void go_forward_and_align() {
  // Variable declarations
  int alignment_gain=4;
  int* side_dist;
  side_dist = get_side_distances();

  //Speed_adj is gain needed to turn motor back on course.
  int speed_adj=alignment_gain*((*(side_dist+1)) - (*(side_dist )));

  left_front_motor.writeMicroseconds(1500 + speed + speed_adj);
  left_rear_motor.writeMicroseconds(1500 + speed + speed_adj);
  right_rear_motor.writeMicroseconds(1500 - speed + speed_adj);
  right_front_motor.writeMicroseconds(1500 - speed + speed_adj);
}

int get_median_of_array(int* array, int size) {
  // Sort an array of readings and return the median value.
  // Key parameters:
  //  array = array of sensor readings
  //  size = no. of elements in the array.
  int i, j, a;
  int copy_of_array[5];

  // Make a duplicate of the array so that sorting won't affect the global array.
  for (i=0; i<size; i++) {
      copy_of_array[i] = *(array+i);
  }

  // Sort array from smallest to largest values.
  for (i=0; i<(size-1); i++) {
      for (j=0; j<(size-1); j++) {
        if (copy_of_array[j] > copy_of_array[j+1]) {
          a = copy_of_array[j+1];
          copy_of_array[j+1] = copy_of_array[j];
          copy_of_array[j] = a;
        }
      }
  }

  // Return median value.
  return copy_of_array[2];
}

/* Return time (ms) required to turn the robot approx.
90 degrees according to the current speed setting*/
int get_turn_delay() {
  return
  // TODO: FINISH this function
  // For speed = 300, use TURN_DELAY = 500; If speed = 150, use 1500.
  #define TURN_DELAY 1000
}

int* get_front_ir_distances() {
  // Return a median value of past 5 front ir sensor readings.
  int i;
  static int front_left_ir[ITERATION_SIZE];
  static int front_right_ir[ITERATION_SIZE];
  static int dist[2];

  //Get 5 new readings
  for (i=0; i<(ITERATION_SIZE-1); i++) {
      front_left_ir[i] = 136894*(pow(analogRead(A4), (-1.157)));//Left sensor
      front_right_ir[i] = 102243*(pow(analogRead(A3), (-1.112)));//Right Sensor
  }


  dist[0] = get_median_of_array(front_left_ir, ITERATION_SIZE);
  dist[1] = get_median_of_array(front_right_ir, ITERATION_SIZE);

  // Get previous value if reading is 0 or negative distance.
  if (dist[0] <= 0) {
    dist[0] = d_front_left;
  }

  if (dist[1] <= 0) {
    dist[1] = d_front_right;
  }
  return dist;
}

int get_sonar_distance() {
    int i, sonar_reading;
    int* sonar_d;

    for (i=0; i<(ITERATION_SIZE-1); i++) {
      sonar_d = ping_location();
      sonar[i] = *(sonar_d+1);
    }

    sonar_reading = get_median_of_array(sonar, ITERATION_SIZE);
    // Get previous value if reading is 0 or negative distance.
    if (sonar_reading <= 0) {
      return d_front_middle;
    } else {
      return sonar_reading;
    }

}

int get_avg_front_distance(){
  // Return side distance from robot to wall. Average of 2 side sensors.
  // First index is front sensor, second index is back sensor.
  return ((d_front_left + d_front_right + d_front_middle)/3);
}

int* get_side_distances() {
  // Return a median value of past 5 side ir sensor readings.
  int i;
  static int side_front_ir[ITERATION_SIZE];
  static int side_rear_ir[ITERATION_SIZE];
  static int dist[2];

  // Get 5 new readings.
  for (i=0; i<(ITERATION_SIZE-1); i++) {
    //The front sensor gives a different reading to the rear (approx 7mm error).
    //The rear sensor is compensated
    side_front_ir[i] = 317756*(pow(analogRead(A1),(-1.172)));
    side_rear_ir[i] = (317756*(pow(analogRead(A2),(-1.172))))+7;
  }

  dist[0] = get_median_of_array(side_front_ir, ITERATION_SIZE);
  dist[1] = get_median_of_array(side_rear_ir, ITERATION_SIZE);
  return dist;
}

int get_avg_side_distance() {
  // Return side distance from robot to wall. Average of 2 side sensors.
  int* side_dist;
  int avg_side_dist;

  side_dist = get_side_distances();
  // First index is front sensor, second index is back sensor.
  avg_side_dist = (*(side_dist) + *(side_dist + 1))/2;
  return avg_side_dist;
}

bool is_side_diff_ok() {
  // Return 1 if sensors are within a reasonable difference of each other. Otherwise false.
  // Return of false means there is an object present near to front or back of robot.
  int* side_dist;
  side_dist = get_side_distances();

  // 70 is acceptable difference between sensors.
  if ((abs(*side_dist - *(side_dist+1))) < 70) {
    return true;
  } else {
    return false;
  }
}

/*----------------------------------------------------------------*/
// OBSTACLE CODE:
/*----------------------------------------------------------------*/
bool is_obstacle_present(){
  bool A,B,C; //Boolean values which indicte whether an obstacle
  // is less than X mm from Left,  mid, right sensors respectively
  int tolerance = OB_LIMIT;

  A = (d_front_left<tolerance);
  B = (d_front_middle<tolerance);
  C = (d_front_right<tolerance);

  // Using truth table
  if(((!A)&&B)||((!A)&&C)||(A&&(!C))){
    ob_counter++;
  }else{
    ob_counter=0;
  }

  if (ob_counter>5) return true;

  return false;
}

int check_diff (int left_val, int right_val){
    //Function checks which value is greater
    //Returns -1 for left, 0 for equal and 1 for right_val
   //tolerance of 20% difference
  int average=(left_val+right_val)/2;

  float tolerance=average* ir_calibration; //tolerance of 20% difference
  int difference=left_val-right_val;

  //If difference is within tolerance return 0 for equal
  if (abs(difference)<tolerance) {
    return 0;
  }

  //If left is greater, return -1
  if (difference>0) {
    return -1;
  }

  //If right is greater, return 1
  if (difference<0) {
    return 1;
  }

  return 2; // Should never happen, error check
}


/*----------------------------------------------------------------*/
// STATE CODE:
/*----------------------------------------------------------------*/

STATE initialising() {
  //Do some initialising
  //Serial1.println("INITIALISING....");
  delay(1000); //One second delay to see the Serial1 string "INITIALISING...."
  //Serial1.println("Enabling Motors...");
  enable_motors();
  enable_servo();
  if (test_enable) return TEST;
  return STARTUP;
}

STATE test() {
  read_Serial11_command();
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  /*--------------------------------------------------------------*/

  //is_ob_in_front();

  /*-----------------------------------------------------------------*/
  /*
  if(is_obstacle_present())
  {
    strafe_right();
  } else{
    stop();
  //  strafe_left();
  }*/

  int max_front_d=d_front_left;
  //int front_dist = get_avg_front_distance();
  //Make front dist the maximum front D

  if(d_front_middle>max_front_d)
  {
    max_front_d=d_front_middle;
  }

  if(d_front_right>max_front_d)
  {
    max_front_d=d_front_right;
  }

  Serial1.println("Left = ");
  Serial1.print(d_front_left);
  Serial1.print("Mid = ");
  Serial1.print(d_front_middle);
  Serial1.print("Right = ");
  Serial1.print(d_front_right);
  Serial1.print("Max = ");
  Serial1.print(max_front_d);

  return TEST;
}

STATE running() {
  fast_flash_double_LED_builtin();
  range_and_speed_settings();


  if (is_battery_voltage__not_OK()) return STOPPED;
  return RUNNING;
}

STATE startup() {
  int avg_side_dist;
  int* side_dist;

  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  if (is_battery_voltage__not_OK()) return STOPPED;

  side_dist = get_side_distances();

  // Align to wall 1
  align();
  if (is_side_diff_ok() == false) {
    //Check which sensor on which side is closer to wall.

    //Checks if sensors are unbalanced which may indicate obstacle.
    //The robot will turn in order to avoid hitting the obstacle.
    if ( (*(side_dist) < *(side_dist+1)) ){
      ccw();
    } else {
        cw();
    }
    // TODO: Redo this turn delay part by using the function with the correct linear trend equation
    delay(TURN_DELAY);
  }
  //Move to the right (Closer to the wall) until the selected distance is reached
  strafe_right();

  // Find distance to wall and start wall following if it is close enough
  avg_side_dist = get_avg_side_distance();

  if ((avg_side_dist < ROBOT_WIDTH) && is_side_diff_ok()) return WALL_FOLLOW;

  return STARTUP;
}

STATE wall_follow() {
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  int* side_dist = get_side_distances();

  //Have integers for each sensor pair
  int pairL=check_diff(d_front_left,d_front_middle);//Check left IR and sonar for which is greater
  int pairR=check_diff(d_front_right,d_front_middle);//Check right IR and sonar

  int max_front_d=d_front_left;
  //int front_dist = get_avg_front_distance();
  //Make front dist the maximum front D

  if(d_front_middle>max_front_d)
  {
    max_front_d=d_front_middle;
  }

  if(d_front_right>max_front_d)
  {
    max_front_d=d_front_right;
  }

  if (is_obstacle_present() == true)
  {
    ob_strafe_time=millis()/10;//Record time starting strafe
    return BEGIN_OBSTACLE_STRAFE;
  }

  // If there is an obstacle to the side, do not reference it. Just go forward for a few iterations.
  if (abs(*(side_dist) - *(side_dist+1)) > 100) {
    forward_counter++;
  }

  if (forward_counter > 0) {
    forward();
    forward_counter++;

    if (forward_counter > 15) {
      // Reset the iteration. Gone past obstacle, go back to forward and align.
      forward_counter = 0;
    }
  } else {
    go_forward_and_align();
  }

  //Check if the robot is within the obsacle detection threshold

  //Potentially change to sonar value insted of max_front_d
  if ( max_front_d <= (OB_LIMIT)){
      return BACK;
  }
  //The robot will not turn unless it is close enough to tell the difference betweent the wall and obstacle
  if ((max_front_d > 0) && (max_front_d <= d_wall)){
    if (max_front_d<OB_LIMIT){
          return CORNER;
    }
  }




  return WALL_FOLLOW;
}

STATE corner() {
  int avg_side_dist;
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  int i = 0;

  //Reset obstacle behind flag
  ob_in_front = false;

  //Turn approximatly 90 deg to line up with next wall
  ccw();
  // TODO: Redo this turn delay part by using the function with the correct linear trend equation
  delay(TURN_DELAY);
  stop();
  delay(500);
  align();

  if (is_battery_voltage__not_OK()) return STOPPED;
  //Strafe right until the robot is desired distance to wall

  if(ob_in_front == false)
  {
    do {
        strafe_left();
        update_and_output();
        Serial1.println("11");
        avg_side_dist = get_avg_side_distance();
    } while (avg_side_dist<d_wall);
  }

  do {
      strafe_right();
      update_and_output();
      Serial1.println("11");
      avg_side_dist = get_avg_side_distance();
  } while (avg_side_dist>d_wall);

  stop();
  for(i = 0; i  < 10; i++)
  {
    update_no_output();
  }

  return WALL_FOLLOW;
}

STATE back(){

  if ((d_front_middle)<(d_wall-100)){
    reverse();

    return BACK;
  }

  return CORNER;
}

STATE begin_obstacle_strafe() {
  // Strafe left to avoid obstacle
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  if (is_battery_voltage__not_OK()) return STOPPED;

  ob_in_front=true;

  if (d_front_right < 300) {
    strafed_past_ob = true;
  }


  //Exit the case when all three front sensors > 240, otherwise keep strafing left.
  if ((d_front_left>240)&&(d_front_right>240)&&(d_front_middle>240)&& strafed_past_ob) {
    stop();
    strafed_past_ob = false;
    //Reset counter
    ob_forward_time= (millis()/10);
    return OBSTACLE;
  } else {
    strafe_left();
    //Timeout from state
    if (((millis()/10)-ob_strafe_time)>300){
      //Reset counter
      ob_forward_time= (millis()/10);
      return OBSTACLE;
    }
    return BEGIN_OBSTACLE_STRAFE;
  }
}

STATE obstacle() {
  // State after strafing left. Go forward and strafe right.
  int* side_dist;
  int front_dist = get_avg_front_distance();
  if (is_battery_voltage__not_OK()) return STOPPED;

  //Have integers for each sensor pair
  int pairL=check_diff(d_front_left,d_front_middle);//Check left IR and sonar for which is greater
  int pairR=check_diff(d_front_right,d_front_middle);//Check right IR and sonar

  //Make front dist the maximum front D
  int max_front_d=d_front_left;
  if(d_front_middle>max_front_d) {
    max_front_d=d_front_middle;
  }
  if(d_front_right>max_front_d) {
    max_front_d=d_front_right;
  }

  forward();
  side_dist = get_side_distances();
  if (*(side_dist) < 220) {
      //Set flag to true, if obstacle is detected by rear sensor
      ob_on_right = true;
  }

  if (is_obstacle_present() == true)
  {
    ob_strafe_time=(millis()/10);
    return BEGIN_OBSTACLE_STRAFE;
  }

  if (ob_on_right && (*(side_dist) > 220)&&(((millis()/10)-ob_forward_time)>170)) {
      // No obstacle detected anymore. Set to false.
      ob_on_right = false;
      stop();
      return END_OBSTACLE_STRAFE;
  }

  if ((max_front_d > 0) && (max_front_d <= d_wall)){
    if ((pairR==0)&&(pairL==0)){
      return CORNER;
    }
  }

  //Time out of obstacle aviod Function
  if (((millis()/10)-ob_forward_time)>200){
    return END_OBSTACLE_STRAFE;
  }

  return OBSTACLE;
}

STATE end_obstacle_strafe() {
  // Avoided obstacle now strafe right.
  int dist;
  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  if (is_battery_voltage__not_OK()) return STOPPED;

  dist = get_avg_side_distance();
  if (dist <= d_wall) {
    // Exit case if returned to original path (before obstacle avoidance).
    return WALL_FOLLOW;
  } else {
    // Continue to strafe to original path.
    strafe_right();
    return END_OBSTACLE_STRAFE;
  }
}

STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;

  disable_motors();
  slow_flash_LED_builtin();

  // Print massage every 500ms
  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    Serial1.println("Lipo voltage too LOW, any lower and the lipo with be damaged");
    Serial1.println("Please Re-charge Lipo");
  }
  // 500ms timed if statement to check lipo and output speed settings
  if (!is_battery_voltage__not_OK()) {
    counter_lipo_voltage_ok++;
    if (counter_lipo_voltage_ok > 20) { // Making sure lipo voltage is stable
      counter_lipo_voltage_ok = 0;
      return RUNNING;
    }
  } else counter_lipo_voltage_ok = 0;
  return STOPPED;
}

/*----------------------------------------------------------------*/
// DEFAULT FUNCTIONS CODE:
/*----------------------------------------------------------------*/
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
    int Lipo_level_cal;
    //the voltage of a LiPo cell depends on its chemistry and varies from about 2.7-3.1 V (discharged) = 620(3.1V Min)
    //to about 4.20-4.25 V (fully charged) = 820(4.1V Max)
    Lipo_level_cal = (analogRead(A0) - 620);
    Lipo_level_cal = Lipo_level_cal * 100;
    Lipo_level_cal = Lipo_level_cal / 200;
    speed += speed_change;
    if (speed > 1000)
      speed = 1000;
    speed_change = 0;
    /*
    Serial1.print("Battery level:");
    Serial1.print(Lipo_level_cal);
    Serial1.println("%");*/
    if (Lipo_level_cal < 0) return false;
  }
  return false;
}

void range_and_speed_settings()
{
  static unsigned long previous_millis;

  if (millis() - previous_millis > 500) {  //500ms timed if statement to check lipo and output speed settings
    previous_millis = millis();
  }

}

void read_Serial11_command()
{
  if (Serial1.available()) {
    char val = Serial1.read();
    Serial1.print("Speed:");
    Serial1.print(speed);
    Serial1.print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        Serial1.println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        Serial1.println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        Serial1.println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        Serial1.println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        Serial1.println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        Serial1.println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        Serial1.println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        Serial1.println("+");
        break;
      default:
        stop();
        Serial1.println("stop");
        break;
    }

  }

}

void disable_motors()
{
  left_front_motor.detach();  // detach the servo on pin left_front to the servo object
  left_rear_motor.detach();  // detach the servo on pin left_rear to the servo object
  right_rear_motor.detach();  // detach the servo on pin right_rear to the servo object
  right_front_motor.detach();  // detach the servo on pin right_front to the servo object

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front to the servo object
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to the servo object
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to the servo object
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to the servo object
}
void stop() //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward()
{
  left_front_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_front_motor.writeMicroseconds(1500 - speed);
}

void reverse ()
{
  left_front_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_front_motor.writeMicroseconds(1500 + speed);
}

void ccw ()
{
  left_front_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_front_motor.writeMicroseconds(1500 - speed);
}

void cw ()
{
  left_front_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_front_motor.writeMicroseconds(1500 + speed);
}

void strafe_left ()
{
  left_front_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_front_motor.writeMicroseconds(1500 - speed);
}

void strafe_right ()
{
  left_front_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_front_motor.writeMicroseconds(1500 + speed);
}

/* Contains all definitions for the rover states */

STATE initialising() {
  enable_motors();
  enable_servo();

  if (RUN_TEST_STATE_ONLY)
    return TEST;
  else:
    return STARTUP;
}

STATE test() {
  read_Serial11_command();
  fast_flash_double_LED_builtin();
  range_and_speed_settings();

  // Add any code here to test it out.
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
  int* d_side_ir_sensors;

  fast_flash_double_LED_builtin();
  range_and_speed_settings();
  if (is_battery_voltage__not_OK()) return STOPPED;

  d_side_ir_sensors = get_side_distances();

  // Align to wall 1
  align();
  if (is_side_diff_ok() == false) {
    //Check which sensor on which side is closer to wall.

    //Checks if sensors are unbalanced which may indicate obstacle.
    //The robot will turn in order to avoid hitting the obstacle.
    if ( (*(d_side_ir_sensors) < *(d_side_ir_sensors+1)) ){
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
  int* d_side_ir_sensors = get_side_distances();

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
  if (abs(*(d_side_ir_sensors) - *(d_side_ir_sensors+1)) > 100) {
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

  // Reverse if too close to obstacle/wall.
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
  int* d_side_ir_sensors;
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
  d_side_ir_sensors = get_side_distances();
  if (*(d_side_ir_sensors) < 220) {
      //Set flag to true, if obstacle is detected by rear sensor
      ob_on_right = true;
  }

  if (is_obstacle_present() == true)
  {
    ob_strafe_time=(millis()/10);
    return BEGIN_OBSTACLE_STRAFE;
  }

  if (ob_on_right && (*(d_side_ir_sensors) > 220)&&(((millis()/10)-ob_forward_time)>170)) {
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

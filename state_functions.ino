/*  Contains all definitions for the rover states
 *  Authors: Nitish Lobo & Theo Drissner-Devine
 */

/* Attach  wheel and front sonar servos variables to the relevant pins.
 * Return next state (STARTUP state).
 */
STATE initialising() {
  enable_motors();
  enable_servo();
	return STARTUP;
}

/* Allow developer to test any new logic, code or algorithm.
 * Also allow anyone to control the rover by sending a serial command.
 * Return same state.
 */
STATE test() {
	enable_motors();
	enable_servo();
	// Allow serial control of rover.
  read_and_execute_serial_command();

  /* Add any code/logic here to test it out. */

  return TEST;
}

/* Get rover to align to the closest wall and avoid all obstacles while doing it.
 * Return next state (either same state or WALL_FOLLOW state).
 */
STATE startup() {
  int d_avg_side;
  int* d_side_ir_sensors;
  d_side_ir_sensors = get_side_distances();

  blink_onboard_led_quickly();

  // Align rover chassis to be paralled to any wall to its right hand side.
  align();
  if (is_rover_parallel_to_wall() == false) {
    // Turn the rover away from the obstacle which is causing the side sensors not give the same reading.
    if ((*(d_side_ir_sensors) < *(d_side_ir_sensors+1))) {
      ccw();
    } else {
      cw();
    }
    delay(TURN_DELAY);
  }
  // Move closer to the wall.
  strafe_right();

  // Start following the wall if the rover is close to and aligned with the wall.
  d_avg_side = get_avg_side_distance();
  if ((d_avg_side < ROBOT_WIDTH) && is_rover_parallel_to_wall())
    return WALL_FOLLOW;

  return STARTUP;
}

/* Drive rover forward whilst keeping it parallel to the wall.
 * Return next state (either BEGIN_OBSTACLE_STRAFE, BACK, CORNER or WALL_FOLLOW).
 */
STATE wall_follow() {
  // Compare front sensors (left: IR sensor, middle: sonar, right: IR sensor).
  int front_left_pair = check_diff(d_front_left, d_front_middle);
  int front_right_pair = check_diff(d_front_right, d_front_middle);

  // Determine the longest distance read from front sensors.
  int d_max_front = d_front_left;
  if(d_front_middle > d_max_front)
    d_max_front = d_front_middle;
  if(d_front_right > d_max_front)
    d_max_front = d_front_right;

  if (is_obstacle_present() == true)
  {
    // Record time starting strafe.
    ob_strafe_time = millis()/10;
    return BEGIN_OBSTACLE_STRAFE;
  }

  // Do not reference the side if any obstacle is present, instead go forward for a few iterations.
  int* d_side_ir_sensors = get_side_distances();
  if (abs(*(d_side_ir_sensors) - *(d_side_ir_sensors + 1)) > 100) {
    forward_counter++;
  }

  if (forward_counter > 0) {
    forward();
    forward_counter++;

    // Gone past the obstacle.
    if (forward_counter > 15)
      forward_counter = 0;
  } else {
    go_forward_and_align();
  }

  // Reverse if too close to obstacle/wall.
  if (d_max_front <= (OB_LIMIT))
    return BACK;

  // Only turn the rover if it is close enough to tell the difference between a wall and an obstacle.
  if ((d_max_front > 0) && (d_max_front <= d_wall) && (d_max_front < OB_LIMIT))
    return CORNER;

  return WALL_FOLLOW;
}

/* Turn rover when it approaches a wall to its front.
 * Rover will not turn if it approaches an obstacle and will instead strafe to avoid hitting it.
 * Return next state (WALL_FOLLOW state).
 */
STATE corner() {
  int d_avg_side;
  int i = 0;
  ob_in_front = false;

  // Turn approximately 90 degrees to align to the next wall.
  ccw();
  delay(TURN_DELAY);
  stop();
  delay(500);
  align();

  // Move rover away from the wall because its 90 degree turn will bring it close to the wall.
  if (ob_in_front == false) {
    do {
      strafe_left();
      update_and_output();
      Serial1.println("11");
      d_avg_side = get_avg_side_distance();
    } while (d_avg_side < d_wall);
  }

  // Bring rover closer to the right wall.
  do {
      strafe_right();
      update_and_output();
      Serial1.println("11");
      d_avg_side = get_avg_side_distance();
  } while (d_avg_side > d_wall);

  stop();

  // Update sensors multiple times to have more sample data points to filter over.
  for (i=0; i<10; i++)
    update_no_output();

  return WALL_FOLLOW;
}

/* Move the rover backwards.
 * Return next state (either the same state or corner).
 */
STATE back() {
  if ((d_front_middle) < (d_wall - 100)) {
    reverse();
    return BACK;
  }
  return CORNER;
}

/* Strafe rover to left to avoid hitting obstacle.
 * Return next state.
 */
STATE begin_obstacle_strafe() {
  ob_in_front = true;
  if (d_front_right < 300)
    strafed_past_ob = true;

  // Exit the case when all three front sensors > 240, otherwise keep strafing left.
  if ((d_front_left > 240) && (d_front_right > 240) && (d_front_middle > 240) && strafed_past_ob) {
    stop();
    strafed_past_ob = false;
    ob_forward_time= (millis()/10);
    return OBSTACLE;
  } else {
    strafe_left();
    // Timeout to next state (ie. assume wrong sensor readings after 3 seconds).
    if (((millis()/10) - ob_strafe_time) > 300) {
      ob_forward_time = (millis()/10);
      return OBSTACLE;
    }
    return BEGIN_OBSTACLE_STRAFE;
  }
}

/* Overtake obstacle (ie. move rover forward past obstacle).
 * Return next state (either BEGIN_OBSTACLE_STRAFE, same state, END_OBSTACLE_STRAFE or CORNER).
 */
STATE obstacle() {
  int* d_side_ir_sensors;
  int front_dist = get_avg_front_distance();

  // Compare front sensors (left: IR sensor, middle: sonar, right: IR sensor).
  int front_left_pair = check_diff(d_front_left, d_front_middle);
  int front_right_pair = check_diff(d_front_right, d_front_middle);

  // Determine the longest distance read from front sensors.
  int d_max_front = d_front_left;
  if(d_front_middle > d_max_front)
    d_max_front = d_front_middle;
  if(d_front_right > d_max_front)
    d_max_front = d_front_right;

  forward();
  d_side_ir_sensors = get_side_distances();
  // Use side sensor to detect whether rover has obstacle to its right.
  if (*(d_side_ir_sensors) < 220)
    ob_on_right = true;

  // Detect whether rover has another obstacle on the front side.
  if (is_obstacle_present() == true)
  {
    ob_strafe_time = (millis()/10);
    return BEGIN_OBSTACLE_STRAFE;
  }

  // No obstacle detected.
  if (ob_on_right && (*(d_side_ir_sensors) > 220) && (((millis()/10) - ob_forward_time) > 170)) {
      ob_on_right = false;
      stop();
      return END_OBSTACLE_STRAFE;
  }

  if ((d_max_front > 0) && (d_max_front <= d_wall)) {
    if ((front_right_pair == 0) && (front_left_pair == 0))
      return CORNER;
  }

  // Timeout to next state (ie. assume wrong sensor readings after 2 seconds).
  if (((millis()/10) - ob_forward_time) > 200)
    return END_OBSTACLE_STRAFE;

  return OBSTACLE;
}

/* Move rover to its original path after overtaking obstacle.
 * Return next state (either WALL_FOLLOW or same state).
 */
STATE end_obstacle_strafe() {
  int dist;
  dist = get_avg_side_distance();
  if (dist <= d_wall) {
    return WALL_FOLLOW;
  } else {
    strafe_right();
    return END_OBSTACLE_STRAFE;
  }
}

/* Stop rover.
 * Return same state.
 */
STATE stopped() {
  disable_motors();
  blink_onboard_led_slowly();
  return STOPPED;
}

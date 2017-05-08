/* Functions to help rover avoid hitting obstacles.
 * Authors: Nitish Lobo & Theo Drissner-Devine.
 */

/* Check whether rover has obstacle in front of it.
 * Return true or false.
 */
bool is_obstacle_present() {
	// Boolean values indicate whether
	// an obstacle is less than X mm from front left, middle & right sensors.
	bool A,B,C;
  int tolerance = OB_LIMIT;
  A = (d_front_left < tolerance);
  B = (d_front_middle < tolerance);
  C = (d_front_right < tolerance);

  // Use truth table logic
  if(((!A) && B) || ((!A) && C) || (A && (!C))) {
    ob_counter++;
  } else {
    ob_counter=0;
  }

	// Return true if rover has observed obstacle for more than 5 times.
	// (ie. filter out outlier distance readings from sensors due to noise).
  if (ob_counter > 5)
    return true;

  return false;
}

/* Check difference between left and right sensors.
 * Input args:
 * 		left_val - distance reading from left IR sensor.
 * 		right_val - distance reading from right IR sensor.
 * Returns:
 * 		-1 if left IR sensor distance reaading is greater than right one.
 * 		0 for equal readings.
 * 		1 if right IR sensor distance reading is greater than left one.
 * 		2 error code.
 */
int check_diff(int left_val, int right_val) {
	// Tolerance of 20% difference
  int average = (left_val + right_val)/2;
  float tolerance = average * ir_calibration;
  int difference = left_val - right_val;

  if (abs(difference)<tolerance)
    return 0;
  if (difference > 0)
    return -1;
  if (difference < 0)
    return 1;
  return 2;
}

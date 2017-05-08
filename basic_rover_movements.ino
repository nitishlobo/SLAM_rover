/* Functions for basic rover movement control.
 * Authors: Nitish Lobo & Theo Drissner-Devine.
 */

/* Configure motors to their corresponding pins. */
void enable_motors() {
  // Attach servo objects to the corresponding pins.
  front_left_motor.attach(front_left);
  back_left_motor.attach(back_left);
  back_right_motor.attach(back_right);
  front_right_motor.attach(front_right);
}

/* Detach motors from their pins and pull pin state to
 * low or high depending on whether pull-down/pull-up resistors are used. */
void disable_motors() {
  // Detach servo objects from the pins.
  front_left_motor.detach();
  front_right_motor.detach();
  back_left_motor.detach();
  back_right_motor.detach();

  // Do not allow floating voltages on the pins.
  pinMode(front_left, INPUT);
  pinMode(back_left, INPUT);
  pinMode(back_right, INPUT);
  pinMode(front_right, INPUT);
}

/* Move omni-directional wheeled rover foward. */
void forward() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 + speed);
  front_right_motor.writeMicroseconds(1500 - speed);
  back_left_motor.writeMicroseconds(1500 + speed);
  back_right_motor.writeMicroseconds(1500 - speed);
}

/* Move omni-directional wheeled rover backwards. */
void reverse() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 - speed);
  front_right_motor.writeMicroseconds(1500 + speed);
  back_left_motor.writeMicroseconds(1500 - speed);
  back_right_motor.writeMicroseconds(1500 + speed);
}

/* Move omni-directional wheeled rover to the left without turning. */
void strafe_left() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 - speed);
  front_right_motor.writeMicroseconds(1500 - speed);
  back_left_motor.writeMicroseconds(1500 + speed);
  back_right_motor.writeMicroseconds(1500 + speed);
}

/* Move omni-directional wheeled rover to the right without turning. */
void strafe_right() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 + speed);
  front_right_motor.writeMicroseconds(1500 + speed);
  back_left_motor.writeMicroseconds(1500 - speed);
  back_right_motor.writeMicroseconds(1500 - speed);
}

/* Spin omni-directional wheeled rover counter-clockwise. */
void ccw() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 - speed);
  front_right_motor.writeMicroseconds(1500 - speed);
  back_left_motor.writeMicroseconds(1500 - speed);
  back_right_motor.writeMicroseconds(1500 - speed);
}

/* Spin omni-directional wheeled rover clockwise. */
void cw() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  // Logic will not work for rovers with normal wheels, only omni-directional wheels.
  front_left_motor.writeMicroseconds(1500 + speed);
  front_right_motor.writeMicroseconds(1500 + speed);
  back_left_motor.writeMicroseconds(1500 + speed);
  back_right_motor.writeMicroseconds(1500 + speed);
}

/* Stop rover wheels. */
void stop() {
  // Continuous servo - 1000 is full speed ccw, 1500 is no movement and 2000 is full speed cw.
  front_left_motor.writeMicroseconds(1500);
  front_right_motor.writeMicroseconds(1500);
  back_left_motor.writeMicroseconds(1500);
  back_right_motor.writeMicroseconds(1500);
}

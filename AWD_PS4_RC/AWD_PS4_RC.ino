// Include the necessary libraries
#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>

// Define constant values
#define PMW_MIN 85    // minimum value for PWM (> 0)
#define PMW_MAX 254   // maximum value for PWM Turbo mode
#define PMW_SLOW 115  // maximum value for PWM Normal mode
#define PMW_BOOST 75  // Normal Mode PWM Accumulator

// Define debug flags (comment out as needed)
// #define DEBUG_MOTORS
// #define DEBUG_CONTROLLER

// Define structures to hold data
struct Motor_Data {  // ... (motor data fields)
  bool motor_left_update;
  bool motor_left_stop;
  uint8_t motor_left_dir;
  uint8_t motor_left_pwm;
  uint8_t motor_left_stop_count;
  bool motor_right_update;
  bool motor_right_stop;
  uint8_t motor_right_dir;
  uint8_t motor_right_pwm;
  uint8_t motor_right_stop_count;
};

struct PS4_Controller_Data {  // ... (controller data fields)
  uint8_t joy_right_PotX;
  uint8_t joy_right_PotY;
  uint8_t joy_left_PotX;
  uint8_t joy_left_PotY;
  uint8_t L2;
  uint8_t R2;
  bool joy_right_Btn;
  bool joy_left_Btn;
  bool L1;
  bool R1;
  bool share_Btn;
  bool options_Btn;
  bool PS_Btn;
  bool triangle_Btn;
  bool square_Btn;
  bool circle_Btn;
  bool cross_Btn;
  bool up_Btn;
  bool down_Btn;
  bool left_Btn;
  bool right_Btn;
  bool show_connected_msg;
  bool show_disconnected_msg;
  // int16_t gyro_x;
  // int16_t gyro_y;
  // int16_t gyro_z;
  int16_t acc_x;
  int16_t acc_y;
  // int16_t acc_z;
};

struct Turbo_Sequense_Data {  // ... (turbo data fields)
  bool ENABLED;
  bool RESETED;
  bool L1;
  bool R1;
  bool triangle_Btn;
  bool down_Btn;
  unsigned long last_pressed_millis;
};

struct Drive_Data {        // ... (drive data fields)
  uint8_t turn_direction;  // 1 --> ternLeft, 2 --> turnRight, 0 --> no turn
  uint8_t steering_value;
  uint8_t drive_direction;  // 1 --> drive forward, 2 --> drive backward, 0 --> stop
  uint8_t throttle_value;
};

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// create USB and and Bluetooth Dongle instances
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);

// Define global variables
Motor_Data motor_data;
Motor_Data prev_motor_data;
PS4_Controller_Data controller_data;
Turbo_Sequense_Data turbo_data;
Drive_Data drive_data;
Color tank_color;
Color car_color;
Color car_trigger_color;
Color car_accel_color;
Color funky_car_color;
Color turbo_color;
int16_t trigger_throttle_value = 0;
unsigned long current_millis = 0;
const uint8_t turbo_sequence_threshold = 300;
const uint8_t mode_count = 4;
const uint8_t motor_count = 2;                   // Number of controlling motors
const uint8_t pwm_pins[2] = { 3, 5 };           // pins 3 and 5 of the arduino uno controlling speed with pwm signal with pins (ENA, ENB) of the L298n drive
const uint8_t forward_dir_pins[2] = { 4, 7 };   // pins 4 and 7 of the arduino uno enable the forward movement pins (IN1,IN3) of the L298n drive
const uint8_t backward_dir_pins[2] = { 8, 6 };  // pins 8 and 6 of the arduino uno enable the backward movement pins (IN2,IN4) of the L298n drive
uint8_t mode = 0;                               //  0 -> tank, 1 -> car, 2 -> car trigger, 3 -> car accel, 4 -> funky car
uint8_t pwm_max = 0;
uint8_t steering_pwm_min = 0;
uint8_t steering_pwm_max = 0;

void setup() {
  Serial.begin(115200);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  for (int motor = 0; motor < motor_count; motor++) {  // Set all the motor control pins to outputs
    pinMode(pwm_pins[motor], OUTPUT);
    pinMode(forward_dir_pins[motor], OUTPUT);
    pinMode(backward_dir_pins[motor], OUTPUT);
  }
  set_default_values();  // Initializes default values for various variables and configuration settings
}

void loop() {
  current_millis = millis();
  Usb.Task();
  if (!check_controller_connection()) return;  // Check if the PS4 controller is connected and handles connection-related messages.
  get_controller_values();                     // Retrieve various controller input values from the PS4 controller.
  if (check_abort()) return;                   // handles aborting the operation if the PS button is pressed on the controller.
  check_turbo_mode();                          // Check for turbo mode and adjust PWM values accordingly.
  set_drive_mode();                            // Set the drive mode based on the controller's button presses.
  run_motors();                                // Control the motors based on the motor data, including direction and PWM values.
  print_debug_data();                          // Depending on the debug flags, this function prints debug data related to motors and controller inputs to the serial console
}

void run_motors() {  // This function controls the motors based on the motor data, including direction and PWM values.
  if (motor_data.motor_left_stop && motor_data.motor_left_stop_count < 10) {
    motor_data.motor_left_stop_count++;
    analogWrite(pwm_pins[0], 254);
    digitalWrite(forward_dir_pins[0], 0);
    digitalWrite(backward_dir_pins[0], 0);
  } else if (motor_data.motor_left_stop) {
    analogWrite(pwm_pins[0], 0);
    motor_data.motor_left_stop = 0;
  } else if (motor_data.motor_left_update) {
    motor_data.motor_left_stop_count = 0;
    analogWrite(pwm_pins[0], motor_data.motor_left_pwm);
    digitalWrite(forward_dir_pins[0], motor_data.motor_left_dir == 1);
    digitalWrite(backward_dir_pins[0], motor_data.motor_left_dir != 1);
  }
  if (motor_data.motor_right_stop && motor_data.motor_right_stop_count < 10) {
    motor_data.motor_right_stop_count++;
    analogWrite(pwm_pins[1], 254);
    digitalWrite(forward_dir_pins[1], 0);
    digitalWrite(backward_dir_pins[1], 0);
  } else if (motor_data.motor_right_stop) {
    analogWrite(pwm_pins[1], 0);
    motor_data.motor_right_stop = 0;
  } else if (motor_data.motor_right_update) {
    motor_data.motor_right_stop_count = 0;
    analogWrite(pwm_pins[1], motor_data.motor_right_pwm);
    digitalWrite(forward_dir_pins[1], motor_data.motor_right_dir == 1);
    digitalWrite(backward_dir_pins[1], motor_data.motor_right_dir != 1);
  }
}

void get_controller_values() {  // This function retrieves various controller input values like joystick positions, button presses, and sensor data from the PS4 controller using the PS4BT library.
  controller_data.joy_left_PotY = PS4.getAnalogHat(LeftHatY);
  controller_data.joy_left_PotX = PS4.getAnalogHat(LeftHatX);
  controller_data.joy_right_PotY = PS4.getAnalogHat(RightHatY);
  // controller_data.joy_right_PotX = PS4.getAnalogHat(RightHatX);
  // controller_data.joy_right_Btn = PS4.getButtonClick(R3);
  // controller_data.joy_left_Btn = PS4.getButtonClick(L3);
  controller_data.L1 = PS4.getButtonClick(L1);
  controller_data.R1 = PS4.getButtonClick(R1);
  controller_data.L2 = PS4.getAnalogButton(L2);
  controller_data.R2 = PS4.getAnalogButton(R2);
  controller_data.options_Btn = PS4.getButtonClick(OPTIONS);
  controller_data.share_Btn = PS4.getButtonClick(SHARE);
  controller_data.PS_Btn = PS4.getButtonClick(PS);
  // controller_data.square_Btn = PS4.getButtonClick(SQUARE);
  controller_data.triangle_Btn = PS4.getButtonClick(TRIANGLE);
  // controller_data.circle_Btn = PS4.getButtonClick(CIRCLE);
  // controller_data.cross_Btn = PS4.getButtonClick(CROSS);
  // controller_data.up_Btn = PS4.getButtonClick(UP);
  controller_data.down_Btn = PS4.getButtonClick(DOWN);
  // controller_data.left_Btn = PS4.getButtonClick(LEFT);
  // controller_data.right_Btn = PS4.getButtonClick(RIGHT);
  if (mode >= 3) {
    // controller_data.gyro_x = PS4.getSensor(gX);
    // controller_data.gyro_y = PS4.getSensor(gY);
    // controller_data.gyro_z = PS4.getSensor(gZ);
    controller_data.acc_x = PS4.getSensor(aX);
    controller_data.acc_y = PS4.getSensor(aY);
    // controller_data.acc_z = PS4.getSensor(aZ);
  } else {
    controller_data.acc_x = 0;
    controller_data.acc_y = 0;
  }
}

//R1+L1+TRIANGLE+DOWN
void check_turbo_mode() {  // This function checks for a specific button sequence on the controller to activate or deactivate turbo mode, adjusting PWM values accordingly.
  // Check if any of the specified buttons are pressed on the controller.
  if (controller_data.triangle_Btn || controller_data.down_Btn || controller_data.L1 || controller_data.R1) {
    turbo_data.RESETED = 1;                           // Set the turbo mode as reset.
    turbo_data.last_pressed_millis = current_millis;  // Record the current time in milliseconds.
    if (!turbo_data.L1)                               // Update the turbo_data flags based on the controller buttons.
      turbo_data.L1 = controller_data.L1;
    if (!turbo_data.R1)
      turbo_data.R1 = controller_data.R1;
    if (!turbo_data.triangle_Btn)
      turbo_data.triangle_Btn = controller_data.triangle_Btn;
    if (!turbo_data.down_Btn)
      turbo_data.down_Btn = controller_data.down_Btn;
    if (turbo_data.L1 && turbo_data.R1 && turbo_data.triangle_Btn && turbo_data.down_Btn) {  // Check if a specific button sequence (L1, R1, Triangle, Down) is pressed simultaneously.
      turbo_data.ENABLED = !turbo_data.ENABLED;                                              // Toggle the turbo mode (ENABLED) flag.
      Serial.print(F("\r\nTURBO MODE: "));                                                   // Print the status of the turbo mode to the serial monitor.
      Serial.print(turbo_data.ENABLED);
      set_controller_color();  // Update the controller color based on the turbo mode status.
      // Adjust PWM values based on the turbo mode status.
      pwm_max = turbo_data.ENABLED ? PMW_MAX : PMW_SLOW;
      steering_pwm_min = !turbo_data.ENABLED ? PMW_MIN + (PMW_BOOST * 1.3) : PMW_MIN;
      steering_pwm_max = !turbo_data.ENABLED ? pwm_max + (PMW_BOOST * 1.3) : pwm_max;
    }
  }
  // Check if a certain amount of time has passed since the last button press and if turbo mode was reset and reset the turbo_data flags.
  if (current_millis - turbo_data.last_pressed_millis > turbo_sequence_threshold && turbo_data.RESETED) {
    turbo_data.L1 = 0;
    turbo_data.R1 = 0;
    turbo_data.triangle_Btn = 0;
    turbo_data.down_Btn = 0;
    turbo_data.RESETED = 0;
  }
}

bool check_controller_connection() {  // This function checks if the PS4 controller is connected and handles connection-related messages.
  // if not connected do nothing
  if (!PS4.connected()) {
    if (controller_data.show_disconnected_msg)
      Serial.print(F("\r\nPS4 is not connected. Waiting ..."));
    controller_data.show_disconnected_msg = 0;
    controller_data.show_connected_msg = 1;
    delay(60);
    return 0;
  } else if (controller_data.show_connected_msg) {
    controller_data.show_connected_msg = 0;
    controller_data.show_disconnected_msg = 1;
    Serial.print(F("\r\nPS4 connected."));
    print_mode();
  }
  return 1;
}

bool check_abort() {  // This function handles aborting the operation if the PS button is pressed on the controller.
  if (controller_data.PS_Btn) {
    Serial.print(F("\r\nABORT!!!"));
    // PS4.disconnect();
    motor_data.motor_left_stop = 1;
    motor_data.motor_right_stop = 1;
    run_motors();
    return 1;
  }
  return 0;
}

void set_drive_mode() {  // This function sets the drive mode based on the controller's button presses. The drive mode determines how the motors respond to controller inputs.
  if (controller_data.options_Btn) {
    mode++;  // Increase the drive mode.
    if (mode > mode_count)
      mode = 0;
    print_mode();
  } else if (controller_data.share_Btn) {
    mode--;  // Decrease the drive mode.
    if (mode < 0)
      mode = mode_count;
    print_mode();
  }
  prev_motor_data = motor_data;
  trigger_throttle_value = controller_data.R2 - controller_data.L2;
  switch (mode) {
    case 0:  // Tank mode controlled with the joysticks
      set_tank_mode();
      break;
    case 1:  // Car mode controlled with the joysticks
      set_direction_data(controller_data.joy_right_PotY, 0, 110, 254, 140);
      set_steering_data(controller_data.joy_left_PotX, 0, 110, 254, 140);
      break;
    case 2:  // Car mode controlled with the R2,L2 triggers and the left joystick
      set_direction_data(trigger_throttle_value, -255, -10, 255, 10);
      set_steering_data(controller_data.joy_left_PotX, 0, 110, 254, 140);
      break;
    case 3:  // Car mode controlled with the R2,L2 triggers and the accelerometer
      set_direction_data(trigger_throttle_value, -255, -10, 255, 10);
      set_steering_data(controller_data.acc_x, -8500, -2000, 8500, 2000);
      break;
    case 4:  // Car mode controlled only with the accelerometer
      set_direction_data(controller_data.acc_y, -8500, -2000, 8500, 2000);
      set_steering_data(controller_data.acc_x, -8500, -2000, 8500, 2000);
      break;
    default:
      break;
  }
  if (mode != 0)
    set_motor_data();
}

// This function sets the motor control mode for a tank-style robot based on joystick input.
void set_tank_mode() {
  if (controller_data.joy_left_PotY > 140) {                                                     // Check the left joystick's vertical position to control the left motor.
    motor_data.motor_left_dir = 2;                                                               // Move the left motor backward
    motor_data.motor_left_pwm = map(controller_data.joy_left_PotY, 140, 254, PMW_MIN, pwm_max);  // Map joystick input to the left motor's PWM value.
    motor_data.motor_left_stop = 0;                                                              // Ensure the left motor is not stopped.
  } else if (controller_data.joy_left_PotY < 110) {
    motor_data.motor_left_dir = 1;                                                             // Move the left motor forward
    motor_data.motor_left_pwm = map(controller_data.joy_left_PotY, 110, 0, PMW_MIN, pwm_max);  // Map joystick input to the left motor's PWM value.
    motor_data.motor_left_stop = 0;                                                            // Ensure the left motor is not stopped.
  } else {                                                                                     // Stop the left motor.
    motor_data.motor_left_dir = 0;
    motor_data.motor_left_pwm = 0;
    motor_data.motor_left_stop = 1;  // Signal that the left motor is stopped.
  }

  if (controller_data.joy_right_PotY > 140) {                                                      // Check the right joystick's vertical position to control the right motor.
    motor_data.motor_right_dir = 2;                                                                // Move the right motor backward.
    motor_data.motor_right_pwm = map(controller_data.joy_right_PotY, 140, 254, PMW_MIN, pwm_max);  // Map joystick input to the right motor's PWM value.
    motor_data.motor_right_stop = 0;                                                               // Ensure the right motor is not stopped.
  } else if (controller_data.joy_right_PotY < 110) {
    motor_data.motor_right_dir = 1;                                                              // Move the right motor forward.
    motor_data.motor_right_pwm = map(controller_data.joy_right_PotY, 110, 0, PMW_MIN, pwm_max);  // Map joystick input to the right motor's PWM value.
    motor_data.motor_right_stop = 0;                                                             // Ensure the right motor is not stopped.
  } else {                                                                                       // Stop the right motor.
    motor_data.motor_right_dir = 0;
    motor_data.motor_right_pwm = 0;
    motor_data.motor_right_stop = 1;  // Signal that the right motor is stopped.
  }
  // If turbo mode is not enabled and the motors have opposite directions, boost the PWM values for easier turning.
  if (!turbo_data.ENABLED && motor_data.motor_right_dir != motor_data.motor_left_dir) {
    if (!motor_data.motor_right_stop)
      motor_data.motor_right_pwm = map(motor_data.motor_right_pwm, PMW_MIN, pwm_max, PMW_MIN + PMW_BOOST, pwm_max + PMW_BOOST);
    if (!motor_data.motor_left_stop)
      motor_data.motor_left_pwm = map(motor_data.motor_left_pwm, PMW_MIN, pwm_max, PMW_MIN + PMW_BOOST, pwm_max + PMW_BOOST);
  }
  // Determine if the motor control values have been updated.
  motor_data.motor_left_update = (prev_motor_data.motor_left_dir != motor_data.motor_left_dir || prev_motor_data.motor_left_pwm != motor_data.motor_left_pwm);
  motor_data.motor_right_update = (prev_motor_data.motor_right_dir != motor_data.motor_right_dir || prev_motor_data.motor_right_pwm != motor_data.motor_right_pwm);
}

// This function sets the drive direction and throttle value based on the input throttle parameter, considering the specified minimum and maximum values with offsets.
void set_direction_data(int16_t throttle_param, int16_t minVal, int16_t offsetMinVal, int16_t maxVal, int16_t offsetMaxVal) {
  if (throttle_param > offsetMaxVal) {
    drive_data.drive_direction = mode == 1 ? 2 : 1;
    drive_data.throttle_value = map(throttle_param, offsetMaxVal, maxVal, 0, 100);
  } else if (throttle_param < offsetMinVal) {
    drive_data.drive_direction = mode == 1 ? 1 : 2;
    drive_data.throttle_value = map(throttle_param, offsetMinVal, minVal, 0, 100);
  } else {
    drive_data.drive_direction = 0;
    drive_data.throttle_value = 0;
  }
}

// This function sets the turn direction and steering value based on the input steering parameter, considering the specified minimum and maximum values with offsets.
void set_steering_data(int16_t steering_param, int16_t minVal, int16_t offsetMinVal, int16_t maxVal, int16_t offsetMaxVal) {
  if (steering_param > offsetMaxVal) {
    drive_data.turn_direction = mode == 4 || mode == 3 ? 2 : 1;
    drive_data.steering_value = map(steering_param, offsetMaxVal, maxVal, 0, 100);
  } else if (steering_param < offsetMinVal) {
    drive_data.turn_direction = mode == 4 || mode == 3 ? 1 : 2;
    drive_data.steering_value = map(steering_param, offsetMinVal, minVal, 0, 100);
  } else {
    drive_data.turn_direction = 0;
    drive_data.steering_value = 0;
  }
}

void set_motor_data() {  // This function sets motor values based on drive and turn directions, as well as throttle and steering values.
  if (drive_data.drive_direction != 0 && drive_data.turn_direction == 0) {
    set_motor_values(drive_data.drive_direction, drive_data.drive_direction, map(drive_data.throttle_value, 0, 100, PMW_MIN, pwm_max), map(drive_data.throttle_value, 0, 100, PMW_MIN, pwm_max));
  } else if (drive_data.drive_direction == 0 && drive_data.turn_direction != 0) {
    if (drive_data.turn_direction == 1)
      set_motor_values(1, 2, map(drive_data.steering_value, 0, 100, steering_pwm_min, steering_pwm_max), map(drive_data.steering_value, 0, 100, PMW_MIN, pwm_max));
    else if (drive_data.turn_direction == 2)
      set_motor_values(2, 1, map(drive_data.steering_value, 0, 100, PMW_MIN, pwm_max), map(drive_data.steering_value, 0, 100, steering_pwm_min, steering_pwm_max));
  } else if (drive_data.drive_direction != 0 && drive_data.turn_direction != 0) {
    if (drive_data.turn_direction == 1) {
      if (drive_data.drive_direction == 1)
        set_motor_values(1, 2, map(drive_data.throttle_value, 0, 100, steering_pwm_min, steering_pwm_max), map(drive_data.steering_value, 0, 200, PMW_MIN, PMW_MIN));
      else
        set_motor_values(2, 1, map(drive_data.throttle_value, 0, 100, steering_pwm_min, steering_pwm_max), map(drive_data.steering_value, 0, 200, PMW_MIN, PMW_MIN));

    } else if (drive_data.turn_direction == 2) {
      if (drive_data.drive_direction == 1)
        set_motor_values(2, 1, map(drive_data.steering_value, 0, 200, PMW_MIN, PMW_MIN), map(drive_data.throttle_value, 0, 100, steering_pwm_min, steering_pwm_max));
      else
        set_motor_values(1, 2, map(drive_data.steering_value, 0, 200, PMW_MIN, PMW_MIN), map(drive_data.throttle_value, 0, 100, steering_pwm_min, steering_pwm_max));
    }
  } else {
    set_motor_values(0, 0, 0, 0);
    motor_data.motor_left_stop = 1;
    motor_data.motor_right_stop = 1;
  }
  motor_data.motor_left_update = (prev_motor_data.motor_left_dir != motor_data.motor_left_dir || prev_motor_data.motor_left_pwm != motor_data.motor_left_pwm);
  motor_data.motor_right_update = (prev_motor_data.motor_right_dir != motor_data.motor_right_dir || prev_motor_data.motor_right_pwm != motor_data.motor_right_pwm);
}

void set_motor_values(uint8_t motor_left_dir, uint8_t motor_right_dir, uint8_t motor_left_pwm, uint8_t motor_right_pwm) {  // This function sets motor control values for left and right motors, considering direction and PWM values.
  motor_data.motor_left_dir = motor_left_dir;
  motor_data.motor_right_dir = motor_right_dir;
  motor_data.motor_left_pwm = motor_left_pwm;
  motor_data.motor_right_pwm = motor_right_pwm;
  motor_data.motor_left_stop = 0;
  motor_data.motor_right_stop = 0;
}

void print_mode() {  // This function prints the current driving mode to the serial console and adjusts the controller's LED color accordingly.
  Serial.print(F("\r\nOptions, changing mode to : "));
  switch (mode) {
    case 0:
      Serial.print(F("Tank"));
      break;
    case 1:
      Serial.print(F("Car_Joystick"));
      break;
    case 2:
      Serial.print(F("Car_Triggers"));
      break;
    case 3:
      Serial.print(F("Car_Accel_Triggers"));
      break;
    case 4:
      Serial.print(F("Funky_Car"));
      break;
    default:
      break;
  }
  set_controller_color();
}

void set_controller_color() {  // This function updates the PS4 controller's LED color based on the selected driving mode and whether turbo mode is enabled.
  if (turbo_data.ENABLED) {
    Color current_color;
    switch (mode) {
      case 0:  // Tank
        current_color.r = (tank_color.r + turbo_color.r) / 2;
        current_color.g = (tank_color.g + turbo_color.g) / 2;
        current_color.b = (tank_color.b + turbo_color.b) / 2;
        break;
      case 1:  //car
        current_color.r = (car_color.r + turbo_color.r) / 2;
        current_color.g = (car_color.g + turbo_color.g) / 2;
        current_color.b = (car_color.b + turbo_color.b) / 2;
        break;
      case 2:
        current_color.r = (car_trigger_color.r + turbo_color.r) / 2;
        current_color.g = (car_trigger_color.g + turbo_color.g) / 2;
        current_color.b = (car_trigger_color.b + turbo_color.b) / 2;
        break;
      case 3:
        current_color.r = (car_accel_color.r + turbo_color.r) / 2;
        current_color.g = (car_accel_color.g + turbo_color.g) / 2;
        current_color.b = (car_accel_color.b + turbo_color.b) / 2;
        break;
      case 4:
        current_color.r = (funky_car_color.r + turbo_color.r) / 2;
        current_color.g = (funky_car_color.g + turbo_color.g) / 2;
        current_color.b = (funky_car_color.b + turbo_color.b) / 2;
        break;
      default:
        break;
    }
    PS4.setLed(current_color.r, current_color.g, current_color.b);
  } else switch (mode) {
      case 0:  // Tank
        PS4.setLed(tank_color.r, tank_color.g, tank_color.b);
        break;
      case 1:  //car
        PS4.setLed(car_color.r, car_color.g, car_color.b);
        break;
      case 2:
        PS4.setLed(car_trigger_color.r, car_trigger_color.g, car_trigger_color.b);
        break;
      case 3:
        PS4.setLed(car_accel_color.r, car_accel_color.g, car_accel_color.b);
        break;
      case 4:
        PS4.setLed(funky_car_color.r, funky_car_color.g, funky_car_color.b);
        break;
      default:
        break;
    }
}

void set_default_values() {  // This function initializes default values for various variables and configuration settings.
  current_millis = 0;
  turbo_data.last_pressed_millis = 0;
  turbo_data.ENABLED = 0;
  turbo_data.RESETED = 1;
  controller_data.show_connected_msg = 1;
  controller_data.show_disconnected_msg = 1;
  motor_data.motor_left_stop_count = 0;
  motor_data.motor_right_stop_count = 0;
  drive_data.throttle_value = 0;
  drive_data.steering_value = 0;
  drive_data.turn_direction = 0;
  drive_data.drive_direction = 0;
  tank_color.r = 0;
  tank_color.g = 0;
  tank_color.b = 254;
  car_color.r = 0;
  car_color.g = 254;
  car_color.b = 0;
  car_trigger_color.r = 254;
  car_trigger_color.g = 0;
  car_trigger_color.b = 0;
  car_accel_color.r = 254;
  car_accel_color.g = 254;
  car_accel_color.b = 0;
  funky_car_color.r = 254;
  funky_car_color.g = 0;
  funky_car_color.b = 254;
  turbo_color.r = 254;
  turbo_color.g = 254;
  turbo_color.b = 254;
  pwm_max = turbo_data.ENABLED ? PMW_MAX : PMW_SLOW;
  steering_pwm_min = !turbo_data.ENABLED ? PMW_MIN + (PMW_BOOST * 1.3) : PMW_MIN;
  steering_pwm_max = !turbo_data.ENABLED ? pwm_max + (PMW_BOOST * 1.3) : pwm_max;
}

void print_debug_data() {  // Depending on the debug flags, this function prints debug data related to motors and controller inputs to the serial console.
#ifdef DEBUG_MOTORS
  if ((motor_data.motor_left_update) || (motor_data.motor_right_update) || (motor_data.motor_right_stop && motor_data.motor_right_stop_count < 20) || (motor_data.motor_left_stop && motor_data.motor_left_stop_count < 20)) {
    Serial.print(F("\r\nMotorLeft: "));
    Serial.print(motor_data.motor_left_pwm);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_dir);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_stop);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_update);
    Serial.print(F(" | MotorRight: "));
    Serial.print(motor_data.motor_right_pwm);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_right_dir);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_right_stop);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_right_update);
  }
#endif
#ifdef DEBUG_CONTROLLER
  Serial.print(F("\r\n"));
  // Serial.print(F("gyro_x: "));
  // Serial.print(controller_data.gyro_x);
  // Serial.print(F(" | gyro_y: "));
  // Serial.print(controller_data.gyro_y);
  // Serial.print(F(" | gyro_z: "));
  // Serial.print(controller_data.gyro_z);

  // Serial.print(F(" | acc_x: "));
  // Serial.print(controller_data.acc_x);
  // Serial.print(F(" | acc_y: "));
  // Serial.print(controller_data.acc_y);
  // Serial.print(F(" | acc_z: "));
  // Serial.print(controller_data.acc_z);

  Serial.print(F(" | jl_Y: "));
  Serial.print(controller_data.joy_left_PotY);
  Serial.print(F(" | jl_X: "));
  Serial.print(controller_data.joy_left_PotX);
  Serial.print(F(" | jr_Y: "));
  Serial.print(controller_data.joy_right_PotY);
  Serial.print(F(" | jr_X: "));
  Serial.print(controller_data.joy_right_PotX);

  // Serial.print(F(" | L3: "));
  // Serial.print(controller_data.joy_left_Btn);
  Serial.print(F(" | L2: "));
  Serial.print(controller_data.L2);
  // Serial.print(F(" | L1: "));
  // Serial.print(controller_data.L1);
  // Serial.print(F(" | R3: "));
  // Serial.print(controller_data.joy_right_Btn);
  Serial.print(F(" | R2: "));
  Serial.print(controller_data.R2);
  // Serial.print(F(" | R1: "));
  // Serial.print(controller_data.R1);

  // Serial.print(F(" | OPTIONS: "));
  // Serial.print(controller_data.options_Btn);
  // Serial.print(F(" | SHARE: "));
  // Serial.print(controller_data.share_Btn);
  // Serial.print(F(" | PS: "));
  // Serial.print(controller_data.PS_Btn);

  // Serial.print(F(" | CROSS: "));
  // Serial.print(controller_data.cross_Btn);
  // Serial.print(F(" | SQUARE: "));
  // Serial.print(controller_data.square_Btn);
  // Serial.print(F(" | CIRCLE: "));
  // Serial.print(controller_data.circle_Btn);
  // Serial.print(F(" | TRIANGLE: "));
  // Serial.print(controller_data.triangle_Btn);

  // Serial.print(F(" | UP: "));
  // Serial.print(controller_data.up_Btn);
  // Serial.print(F(" | DOWN: "));
  // Serial.print(controller_data.down_Btn);
  // Serial.print(F(" | LEFT: "));
  // Serial.print(controller_data.left_Btn);
  // Serial.print(F(" | RIGHT: "));
  // Serial.print(controller_data.right_Btn);
#endif
}
// Include the necessary libraries
#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>

// Define constant values
#define PWM_MIN 30             // minimum value for PWM (> 0)
#define PWM_MAX 254            // maximum value for PWM Turbo mode
#define PWM_SLOW 100           // maximum value for PWM Normal mode
#define PWM_BOOST 5            // maximum value for PWM Normal mode
#define PWM_STEERING_BOOST 60  // Normal Mode PWM Accumulator for steering wheels

// Define debug flags (comment out as needed)
// #define DEBUG_MOTORS
// #define DEBUG_CONTROLLER
// #define DEBUG_DRIVE_DATA

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
  bool left_Btn;
  unsigned long last_pressed_millis;
};

struct Rumble_Data {
  bool ENABLED;
  uint16_t delay;
  unsigned long last_enabled_millis;
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
Rumble_Data rumble_data;
Color tank_color;
Color car_color;
Color car_trigger_color;
Color car_accel_color;
Color funky_car_color;
Color turbo_color;
Color current_color;
int16_t trigger_throttle_value = 0;
unsigned long current_millis = 0;
const uint8_t turbo_sequence_threshold = 300;
const uint8_t mode_count = 4;
const uint8_t motor_count = 2;                  // Number of controlling motors
const uint8_t pwm_pins[2] = { 6, 5 };           // pins 6 and 5  [980 Hz PWM] of the arduino uno controlling speed with pwm signal with pins (ENA, ENB) of the L298n drive
const uint8_t forward_dir_pins[2] = { 8, 3 };   // pins 8 and 3 of the arduino uno enable the forward movement pins (IN1,IN3) of the L298n drive
const uint8_t backward_dir_pins[2] = { 7, 4 };  // pins 7 and 4 of the arduino uno enable the backward movement pins (IN2,IN4) of the L298n drive
int8_t mode = 0;                                //   0 -> car, 1 -> car trigger, 2 -> car accel, 3 -> funky car 4 -> tank
int8_t speed_mode = 0;
uint8_t pwm_max = 0;
uint8_t steering_pwm_min = 0;
uint8_t steering_pwm_max = 0;
uint8_t steering_pwm = 0;  // in case of steering  the one pair of wheels should run in the oposite direction of the rover's moving direction with slower speed in order to simulate smooth turning   ( this is not applied on the tank mode)
uint8_t throttle_steering_pwm = 0;
uint8_t throttle_pwm = 0;


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
  check_speed_mode();                          // Check for speed mode and adjust PWM values accordingly.
  set_drive_mode();                            // Set the drive mode based on the controller's button presses.
  run_motors();                                // Control the motors based on the motor data, including direction and PWM values.
  print_debug_data();                          // Depending on the debug flags, this function prints debug data related to motors and controller inputs to the serial console
  check_kill_rumble();
}

void run_motors() {  // This function controls the motors based on the motor data, including direction and PWM values.
  if (motor_data.motor_left_stop && motor_data.motor_left_update) {
    analogWrite(pwm_pins[0], 254);
    digitalWrite(forward_dir_pins[0], 0);
    digitalWrite(backward_dir_pins[0], 0);
  } else if (motor_data.motor_left_update) {
    analogWrite(pwm_pins[0], motor_data.motor_left_pwm);
    digitalWrite(forward_dir_pins[0], motor_data.motor_left_dir == 1);
    digitalWrite(backward_dir_pins[0], motor_data.motor_left_dir != 1);
  }
  if (motor_data.motor_right_stop && motor_data.motor_right_update) {
    analogWrite(pwm_pins[1], 254);
    digitalWrite(forward_dir_pins[1], 0);
    digitalWrite(backward_dir_pins[1], 0);
  } else if (motor_data.motor_right_update) {
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
  controller_data.up_Btn = PS4.getButtonClick(UP);
  controller_data.down_Btn = PS4.getButtonClick(DOWN);
  controller_data.left_Btn = PS4.getButtonClick(LEFT);
  controller_data.right_Btn = PS4.getButtonClick(RIGHT);
  if (mode == 2 || mode == 3) {
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

//R1+L1+TRIANGLE+LEFT
void check_turbo_mode() {  // This function checks for a specific button sequence on the controller to activate or deactivate turbo mode, adjusting PWM values accordingly.
  // Check if any of the specified buttons are pressed on the controller.
  if (controller_data.triangle_Btn || controller_data.left_Btn || controller_data.L1 || controller_data.R1) {
    turbo_data.RESETED = 1;                           // Set the turbo mode as reset.
    turbo_data.last_pressed_millis = current_millis;  // Record the current time in milliseconds.
    if (!turbo_data.L1)                               // Update the turbo_data flags based on the controller buttons.
      turbo_data.L1 = controller_data.L1;
    if (!turbo_data.R1)
      turbo_data.R1 = controller_data.R1;
    if (!turbo_data.triangle_Btn)
      turbo_data.triangle_Btn = controller_data.triangle_Btn;
    if (!turbo_data.left_Btn)
      turbo_data.left_Btn = controller_data.left_Btn;
    if (turbo_data.L1 && turbo_data.R1 && turbo_data.triangle_Btn && turbo_data.left_Btn) {  // Check if a specific button sequence (L1, R1, Triangle, Down) is pressed simultaneously.
      turbo_data.ENABLED = !turbo_data.ENABLED;                                              // Toggle the turbo mode (ENABLED) flag.
      Serial.print(F("\r\nTURBO MODE: "));                                                   // Print the status of the turbo mode to the serial monitor.
      Serial.print(turbo_data.ENABLED);
      if (turbo_data.ENABLED) {
        PS4.setLedFlash(10, 5);  // Set it to blink rapidly
        set_rumble_on(254, 2000);
      } else {
        PS4.setLedFlash(0, 0);  // Turn off blinking
        PS4.setRumbleOn(0, 0);
      }
      // Adjust PWM values based on the turbo mode status.
      pwm_max = turbo_data.ENABLED ? PWM_MAX : PWM_SLOW;
      steering_pwm_min = !turbo_data.ENABLED ? PWM_MIN + PWM_STEERING_BOOST : PWM_MIN;
      steering_pwm_max = !turbo_data.ENABLED ? pwm_max + PWM_STEERING_BOOST : PWM_MAX;
      set_controller_color();  // Update the controller color based on the turbo mode status.
    }
  }
  // Check if a certain amount of time has passed since the last button press and if turbo mode was reset and reset the turbo_data flags.
  if (current_millis - turbo_data.last_pressed_millis > turbo_sequence_threshold && turbo_data.RESETED) {
    turbo_data.L1 = 0;
    turbo_data.R1 = 0;
    turbo_data.triangle_Btn = 0;
    turbo_data.left_Btn = 0;
    turbo_data.RESETED = 0;
    set_controller_color();
  }
}

void check_speed_mode() {
  if (turbo_data.ENABLED) return;  // If Turbo Mode is enabled, exit the function.
  if (controller_data.down_Btn || controller_data.up_Btn) {
    speed_mode += controller_data.up_Btn ? 1 : -1;
    if (speed_mode >= 6) {  
      speed_mode = 6;
      set_rumble_on(254, 1000); // Enable rumble feedback and set it to a high intensity.
    } else if (speed_mode <= 0) {
      speed_mode = 0;
      set_rumble_on(150, 1000); // Enable rumble feedback and set it to a medium intensity.
    }
    pwm_max = PWM_SLOW;
    pwm_max += speed_mode * PWM_BOOST;
    set_controller_color(); // Update the LED color on the controller to reflect the speed mode.
    Serial.print(F("\r\nSPEED MODE: "));  // Print the status of the speed mode to the serial monitor.
    Serial.print(speed_mode);
  }
}

void check_kill_rumble() {
  if (rumble_data.ENABLED && current_millis - rumble_data.last_enabled_millis > rumble_data.delay) {
    rumble_data.ENABLED = false; // Disable rumble feedback.
    PS4.setRumbleOn(0, 0);// Turn off rumble.
  }
}

void set_rumble_on(uint8_t rumble_value, uint16_t delay) {
  PS4.setRumbleOn(0, rumble_value); // Enable rumble feedback with the specified intensity.
  rumble_data.ENABLED = true; // Mark rumble feedback as enabled.
  rumble_data.last_enabled_millis = current_millis; // Record the current time.
  rumble_data.delay = delay; // Set the delay duration for rumble feedback.
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
    set_controller_color();
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

// This function sets the motor control mode for a tank-style robot based on joystick input.
void set_tank_mode() {
  if (controller_data.joy_left_PotY > 140) {                                                     // Check the left joystick's vertical position to control the left motor.
    motor_data.motor_left_dir = 2;                                                               // Move the left motor backward
    motor_data.motor_left_pwm = map(controller_data.joy_left_PotY, 140, 254, PWM_MIN, pwm_max);  // Map joystick input to the left motor's PWM value.
    motor_data.motor_left_stop = 0;                                                              // Ensure the left motor is not stopped.
  } else if (controller_data.joy_left_PotY < 110) {
    motor_data.motor_left_dir = 1;                                                             // Move the left motor forward
    motor_data.motor_left_pwm = map(controller_data.joy_left_PotY, 110, 0, PWM_MIN, pwm_max);  // Map joystick input to the left motor's PWM value.
    motor_data.motor_left_stop = 0;                                                            // Ensure the left motor is not stopped.
  } else {                                                                                     // Stop the left motor.
    motor_data.motor_left_dir = 0;
    motor_data.motor_left_pwm = 0;
    motor_data.motor_left_stop = 1;  // Signal that the left motor is stopped.
  }

  if (controller_data.joy_right_PotY > 140) {                                                      // Check the right joystick's vertical position to control the right motor.
    motor_data.motor_right_dir = 2;                                                                // Move the right motor backward.
    motor_data.motor_right_pwm = map(controller_data.joy_right_PotY, 140, 254, PWM_MIN, pwm_max);  // Map joystick input to the right motor's PWM value.
    motor_data.motor_right_stop = 0;                                                               // Ensure the right motor is not stopped.
  } else if (controller_data.joy_right_PotY < 110) {
    motor_data.motor_right_dir = 1;                                                              // Move the right motor forward.
    motor_data.motor_right_pwm = map(controller_data.joy_right_PotY, 110, 0, PWM_MIN, pwm_max);  // Map joystick input to the right motor's PWM value.
    motor_data.motor_right_stop = 0;                                                             // Ensure the right motor is not stopped.
  } else {                                                                                       // Stop the right motor.
    motor_data.motor_right_dir = 0;
    motor_data.motor_right_pwm = 0;
    motor_data.motor_right_stop = 1;  // Signal that the right motor is stopped.
  }
  // If turbo mode is not enabled and the motors have opposite directions, boost the PWM values for easier turning.
  if (!turbo_data.ENABLED && motor_data.motor_right_dir != motor_data.motor_left_dir) {
    if (!motor_data.motor_right_stop)
      motor_data.motor_right_pwm = map(motor_data.motor_right_pwm, PWM_MIN, pwm_max, PWM_MIN + PWM_STEERING_BOOST, pwm_max + PWM_STEERING_BOOST);
    if (!motor_data.motor_left_stop)
      motor_data.motor_left_pwm = map(motor_data.motor_left_pwm, PWM_MIN, pwm_max, PWM_MIN + PWM_STEERING_BOOST, pwm_max + PWM_STEERING_BOOST);
  }
  // Determine if the motor control values have been updated.
  motor_data.motor_left_update = (prev_motor_data.motor_left_dir != motor_data.motor_left_dir || prev_motor_data.motor_left_pwm != motor_data.motor_left_pwm);
  motor_data.motor_right_update = (prev_motor_data.motor_right_dir != motor_data.motor_right_dir || prev_motor_data.motor_right_pwm != motor_data.motor_right_pwm);
}

void set_drive_mode() {  // This function sets the drive mode based on the controller's button presses. The drive mode determines how the motors respond to controller inputs.
  if (controller_data.options_Btn) {
    mode++;  // Increase the drive mode.
    if (mode > mode_count)
      mode = 0;
    speed_mode = 0;
  } else if (controller_data.share_Btn) {
    mode--;  // Decrease the drive mode.
    if (mode < 0)
      mode = mode_count;
  }
  if (controller_data.options_Btn || controller_data.share_Btn) {
    set_rumble_on(254, 600);
    PS4.setLedFlash(0, 0);  // Turn off blinking
    turbo_data.ENABLED = false;
    turbo_data.RESETED = 1;
    turbo_data.last_pressed_millis = current_millis;
    // RESET PWM values
    pwm_max = PWM_SLOW;
    steering_pwm_min = PWM_MIN + PWM_STEERING_BOOST;
    steering_pwm_max = PWM_SLOW + PWM_STEERING_BOOST;
    speed_mode = 0;
    set_controller_color(); // Update the LED color on the controller.
    print_mode(); // Print the current drive mode to the console.
  }
  prev_motor_data = motor_data;
  trigger_throttle_value = controller_data.R2 - controller_data.L2;
  switch (mode) {
    case 0:  // Car mode controlled with the joysticks
      set_direction_data(controller_data.joy_right_PotY, 0, 110, 254, 140);
      set_steering_data(controller_data.joy_left_PotX, 0, 105, 254, 145);
      set_motor_data();
      break;
    case 1:  // Car mode controlled with the R2,L2 triggers for throttle and the left joystick for steering.
      set_direction_data(trigger_throttle_value, -255, -10, 255, 10);
      set_steering_data(controller_data.joy_left_PotX, 0, 105, 254, 145);
      set_motor_data();
      break;
    case 2:  // Car mode controlled with the R2,L2 for throttle triggers and the accelerometer for steering.
      set_direction_data(trigger_throttle_value, -255, -10, 255, 10);
      set_steering_data(controller_data.acc_x, -8500, -2000, 8500, 2000);
      set_motor_data();
      break;
    case 3:  // Car mode controlled only with the accelerometer
      set_direction_data(controller_data.acc_y, -8500, -2000, 8500, 2000);
      set_steering_data(controller_data.acc_x, -8500, -2000, 8500, 2000);
      set_motor_data();
      break;
    case 4:  // Tank mode controlled with the joysticks
      set_tank_mode();
      break;
    default:
      break;
  }
}

// This function sets the drive direction and throttle value based on the input throttle parameter, considering the specified minimum and maximum values with offsets.
void set_direction_data(int16_t throttle_param, int16_t minVal, int16_t offsetMinVal, int16_t maxVal, int16_t offsetMaxVal) {
  if (throttle_param > offsetMaxVal) {
    drive_data.drive_direction = mode == 0 ? 2 : 1;
    drive_data.throttle_value = map(throttle_param, offsetMaxVal, maxVal, 0, 100);
  } else if (throttle_param < offsetMinVal) {
    drive_data.drive_direction = mode == 0 ? 1 : 2;
    drive_data.throttle_value = map(throttle_param, offsetMinVal, minVal, 0, 100);
  } else {
    drive_data.drive_direction = 0;
    drive_data.throttle_value = 0;
  }
}

// This function sets the turn direction and steering value based on the input steering parameter, considering the specified minimum and maximum values with offsets.
void set_steering_data(int16_t steering_param, int16_t minVal, int16_t offsetMinVal, int16_t maxVal, int16_t offsetMaxVal) {
  if (steering_param > offsetMaxVal) {
    drive_data.turn_direction = mode == 3 || mode == 2 ? 2 : 1;
    drive_data.steering_value = map(steering_param, offsetMaxVal, maxVal, 0, 100);
  } else if (steering_param < offsetMinVal) {
    drive_data.turn_direction = mode == 3 || mode == 2 ? 1 : 2;
    drive_data.steering_value = map(steering_param, offsetMinVal, minVal, 0, 100);
  } else {
    drive_data.turn_direction = 0;
    drive_data.steering_value = 0;
  }
}

void set_motor_data() {  // This function sets motor values based on drive and turn directions, as well as throttle and steering values.
  if (drive_data.drive_direction != 0 && drive_data.turn_direction == 0) {
    throttle_pwm = map(drive_data.throttle_value, 0, 100, PWM_MIN, pwm_max);
    set_motor_values(drive_data.drive_direction, throttle_pwm, drive_data.drive_direction, throttle_pwm);
  } else if (drive_data.drive_direction == 0 && drive_data.turn_direction != 0) {
    steering_pwm = map(drive_data.steering_value, 0, 100, PWM_MIN, steering_pwm_max);
    if (drive_data.turn_direction == 1)
      set_motor_values(1, steering_pwm, 2, steering_pwm);
    else
      set_motor_values(2, steering_pwm, 1, steering_pwm);
  } else if (drive_data.turn_direction != 0) {
    throttle_steering_pwm = map(drive_data.throttle_value, 0, 100, steering_pwm_min, steering_pwm_max);
    steering_pwm = map(drive_data.steering_value, 0, 100, 0, throttle_steering_pwm - 50);
    if (drive_data.turn_direction == 1) {
      if (drive_data.drive_direction == 2)
        set_motor_values(2, steering_pwm, 1, throttle_steering_pwm);
      else if (drive_data.drive_direction == 1)
        set_motor_values(1, throttle_steering_pwm, 2, steering_pwm);
    } else if (drive_data.turn_direction == 2) {
      if (drive_data.drive_direction == 2)
        set_motor_values(1, throttle_steering_pwm, 2, steering_pwm);
      else if (drive_data.drive_direction == 1)
        set_motor_values(2, steering_pwm, 1, throttle_steering_pwm);
    }
  } else {
    set_motor_values(0, 0, 0, 0);
    motor_data.motor_left_stop = 1;
    motor_data.motor_right_stop = 1;
  }
  motor_data.motor_left_update = (prev_motor_data.motor_left_dir != motor_data.motor_left_dir || prev_motor_data.motor_left_pwm != motor_data.motor_left_pwm);
  motor_data.motor_right_update = (prev_motor_data.motor_right_dir != motor_data.motor_right_dir || prev_motor_data.motor_right_pwm != motor_data.motor_right_pwm);
}

void set_motor_values(uint8_t motor_left_dir, uint8_t motor_left_pwm, uint8_t motor_right_dir, uint8_t motor_right_pwm) {  // This function sets motor control values for left and right motors, considering direction and PWM values.
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
      Serial.print(F("Car_Joystick"));
      break;
    case 1:
      Serial.print(F("Car_Triggers"));
      break;
    case 2:
      Serial.print(F("Car_Accel_Triggers"));
      break;
    case 3:
      Serial.print(F("Funky_Car"));
      break;
    case 4:
      Serial.print(F("Tank"));
      break;
    default:
      break;
  }
}

void set_controller_color() {  // This function updates the PS4 controller's LED color based on the selected driving mode and whether turbo mode is enabled.
  switch (mode) {
    case 0:  //car
      if (turbo_data.ENABLED)
        set_turbo_color(car_color.r, car_color.g, car_color.b);
      else
        set_speed_color(car_color.r, car_color.g, car_color.b);
      break;
    case 1:
      if (turbo_data.ENABLED)
        set_turbo_color(car_trigger_color.r, car_trigger_color.g, car_trigger_color.b);
      else
        set_speed_color(car_trigger_color.r, car_trigger_color.g, car_trigger_color.b);
      break;
    case 2:
      if (turbo_data.ENABLED)
        set_turbo_color(car_accel_color.r, car_accel_color.g, car_accel_color.b);
      else
        set_speed_color(car_accel_color.r, car_accel_color.g, car_accel_color.b);
      break;
    case 3:
      if (turbo_data.ENABLED)
        set_turbo_color(funky_car_color.r, funky_car_color.g, funky_car_color.b);
      else
        set_speed_color(funky_car_color.r, funky_car_color.g, funky_car_color.b);
      break;
    case 4:  // Tank
      if (turbo_data.ENABLED)
        set_turbo_color(tank_color.r, tank_color.g, tank_color.b);
      else
        set_speed_color(tank_color.r, tank_color.g, tank_color.b);
      break;
    default:
      break;
  }
  PS4.setLed(current_color.r, current_color.g, current_color.b);
}

void set_turbo_color(uint8_t r, uint8_t g, uint8_t b) {
  current_color.r = (r + turbo_color.r > 254) ? 254 : r + turbo_color.r;
  current_color.g = (g + turbo_color.g > 254) ? 254 : g + turbo_color.g;
  current_color.b = (b + turbo_color.b > 254) ? 254 : b + turbo_color.b;
}

void set_speed_color(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t speed_acc = speed_mode * 1.5;
  current_color.r = ((r + speed_acc) > 254) ? 254 : r + speed_acc;
  current_color.g = ((g + speed_acc) > 254) ? 254 : g + speed_acc;
  current_color.b = ((b + speed_acc) > 254) ? 254 : b + speed_acc;
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
  turbo_color.r = 20;
  turbo_color.g = 20;
  turbo_color.b = 20;
  rumble_data.ENABLED = false;
  rumble_data.last_enabled_millis = current_millis;
  rumble_data.delay = 1500;
  pwm_max = turbo_data.ENABLED ? PWM_MAX : PWM_SLOW;
  steering_pwm_min = !turbo_data.ENABLED ? PWM_MIN + PWM_STEERING_BOOST : PWM_MIN;
  steering_pwm_max = !turbo_data.ENABLED ? pwm_max + PWM_STEERING_BOOST : PWM_MAX;
}

void print_debug_data() {  // Depending on the debug flags, this function prints debug data related to motors and controller inputs to the serial console.
#ifdef DEBUG_DRIVE_DATA
  if ((motor_data.motor_left_update) || (motor_data.motor_right_update)) {
    Serial.print(F("\r\nD: "));
    Serial.print(drive_data.drive_direction);
    Serial.print(F(", "));
    Serial.print(drive_data.throttle_value);
    Serial.print(F("S:"));
    Serial.print(drive_data.turn_direction);
    Serial.print(F(", "));
    Serial.print(drive_data.steering_value);
  }
#endif
#ifdef DEBUG_MOTORS
  if ((motor_data.motor_left_update) || (motor_data.motor_right_update)) {
    Serial.print(F("\r\nL: "));
    Serial.print(motor_data.motor_left_pwm);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_dir);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_stop);
    Serial.print(F(", "));
    Serial.print(motor_data.motor_left_update);
    Serial.print(F(" | R: "));
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
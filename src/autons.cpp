#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <type_traits>

/////
// For installation, upgrading, documentations and tutorials, check out our
// website! https://ez-robotics.github.io/EZ-Template/
/////z

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;
const int SAFE_DRIVE_SPEED = 60;
const int SAFE_TURN_SPEED = 50;
const int MAX_SPEED = 127;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(3.4, 0, 20);
  chassis.pid_drive_constants_set(15, 0, 5);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(200_ms, 3_deg, 300_ms, 7_deg, 750_ms,
                                      750_ms);
  chassis.pid_swing_exit_condition_set(200_ms, 3_deg, 300_ms, 7_deg, 750_ms,
                                       750_ms);
  chassis.pid_drive_exit_condition_set(200_ms, 0.5_in, 300_ms, 1.5_in, 750_ms,
                                       750_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}

void tuning_exit_conditions() {
  chassis.pid_turn_exit_condition_set(3000_ms, 3_deg, 5000_ms, 7_deg, 7500_ms,
                                      750_ms);
  chassis.pid_swing_exit_condition_set(3000_ms, 3_deg, 5000_ms, 7_deg, 7500_ms,
                                       750_ms);
  chassis.pid_drive_exit_condition_set(3000_ms, 0.5_in, 5000_ms, 1.5_in,
                                       7500_ms, 750_ms);
}

void adjustment_conditions() {
  chassis.pid_drive_constants_set(40, 0, 5);
  chassis.pid_turn_constants_set(10, 0, 20);
  chassis.pid_swing_constants_set(20, 0, 30);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a
  // slew at the start of drive motions for slew, only enable it when the drive
  // distance is greater then the slew distance + a few inches

  tuning_exit_conditions();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining
  // distance at a max speed of 30
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(
      30); // After driving 6 inches at DRIVE_SPEED, the robot will go the
           // remaining distance at 30 speed
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches, the robot will travel the remaining
  // distance at a max speed of 30
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(
      30); // After driving 6 inches at DRIVE_SPEED, the robot will go the
           // remaining distance at 30 speed
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this
  // allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .
void defensive_side_safe() {
  // Defense V1
  intake.move_velocity(-550);
  pros::delay(300);

  // setup
  chassis.pid_turn_set(-55_deg, SAFE_TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(22.167_in, SAFE_DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(145.008_deg, SAFE_TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(11_in, SAFE_DRIVE_SPEED);
  front_wings.set_value(true);
  chassis.pid_wait();

  // double flick
  chassis.pid_turn_set(95.118_deg, SAFE_TURN_SPEED);
  chassis.pid_wait();

  // go to bar
  default_constants();
  chassis.pid_drive_set(38.858_in, DRIVE_SPEED);
  front_wings.set_value(false);
  chassis.pid_wait();

  adjustment_conditions();
  chassis.pid_turn_set(90_deg, SAFE_TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(2_in, SAFE_DRIVE_SPEED);
  chassis.pid_wait();
  front_wings.set_value(true);

  chassis.pid_drive_set(1.5_in, SAFE_DRIVE_SPEED);
  chassis.pid_wait();
}

// defensive mid roosh
void defensive_side_mid_rush() {
  // flick at beggining
  front_wings.set_value(true);

  // move to mid triball
  adjustment_conditions();
  chassis.pid_turn_set(11.31_deg, MAX_SPEED);
  chassis.pid_wait();

  default_constants();
  chassis.pid_drive_set(46.6_in, MAX_SPEED);
  chassis.pid_wait_until(10);
  front_wings.set_value(false);
  chassis.pid_wait_until(45);
  intake.move_velocity(600);
  chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  chassis.pid_speed_max_set(127);

  default_constants();
  // remove triballs
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(10_in, MAX_SPEED);
  front_wings.set_value(true);
  chassis.pid_wait();
  intake.move_velocity(-600);
  pros::delay(500);

  // go to descore
  chassis.pid_drive_set(-10_in, MAX_SPEED);
  front_wings.set_value(false);
  chassis.pid_wait();

  chassis.pid_turn_set(11.31_deg, MAX_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-46.6_in, DRIVE_SPEED);
  chassis.pid_wait();

  // run safe auto
  defensive_side_safe();
}

// safe 6ball
void safe_6_ball() {
  // set heading
  // chassis.drive_imu_reset(-90);

  front_wings.set_value(true);
  pros::delay(250);
  front_wings.set_value(false);

  // get bottom triballD)
  chassis.pid_turn_set(-90_deg, TURN_SPEED - 30);
  chassis.pid_wait();

  chassis.pid_drive_set(29_in, DRIVE_SPEED);
  intake.move_velocity(600);
  chassis.pid_wait();

  // ripoff descore
  default_constants();
  chassis.pid_drive_set(-34_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(60_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_wait();

  chassis.pid_drive_set(17_in, DRIVE_SPEED);
  front_wings.set_value(true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // side push
  intake.move_velocity(-600);
  chassis.pid_drive_set(11.073_in, DRIVE_SPEED);
  chassis.pid_wait();
  front_wings.set_value(false);

  // Second Side push
  chassis.pid_drive_set(-9_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(7.5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.move_velocity(-600);
  chassis.pid_drive_set(10_in, MAX_SPEED);
  chassis.pid_wait();

  adjustment_conditions();
  chassis.pid_drive_set(-3_in, MAX_SPEED);
  chassis.pid_wait();

  intake.move_velocity(0);
  chassis.drive_set(127, 127);
  pros::delay(300);
  chassis.drive_set(0, 0);

  // First triball
  default_constants();
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-75_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(50.509_in, DRIVE_SPEED);
  intake.move_velocity(600);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(70_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.move_velocity(-600);
  pros::delay(400);

  // second triball
  chassis.pid_turn_set(-30_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(12);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();

  chassis.pid_drive_set(-8_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  front_wings.set_value(true);
  intake.move_velocity(-600);
  chassis.pid_drive_set(40_in, MAX_SPEED);
  chassis.pid_wait();
}
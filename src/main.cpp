#include "main.h"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

/////
// For installation, upgrading, documentations and tutorials, check out our
// website! https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is used as the sensor
    {11, 12, 13}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is used as the sensor
    ,
    {-18, -19, -20}

    // IMU Port
    ,
    1

    // Wheel Diameter (Remember, 4" wheels without screw holes are
    // actually 4.125!)
    ,
    2.75

    // Cartridge RPM
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be
    // 84/36 which is 2.333 eg. if your drive is 60:36 where the 36t is powered,
    // your RATIO would be 60/36 which is 0.6 eg. if your drive is 36:60 where
    // the 60t is powered, your RATIO would be 36/60 which is 0.6
    ,
    1);

// intake
pros::MotorGroup intake({17, -16});

// front wings
bool front_wings_out = false;
pros::ADIDigitalOut front_wings('a');

// intake lifter
bool intake_lifted = true;
pros::ADIDigitalOut intake_lifter('h');

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(
      true); // Enables modifying the controller curve with buttons on the
             // joysticks
  chassis.opcontrol_drive_activebrake_set(
      0); // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(
      2.5, 0); // Defaults for curve. If using tank, only the first parameter is
             // used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!
  chassis.pid_tuner_print_terminal_set(true);

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.opcontrol_curve_buttons_left_set
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("Defensive Side Safe", defensive_side_safe),
      Auton("Defensive Side Mid Rush", defensive_side_mid_rush),
      Auton("Safe 6 ball", safe_6_ball),
      Auton("Drive and Turn\n\nSlow down during drive.",
            wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently "
            "if interfered or not.",
            interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");

  // Init other motors
  intake.set_gearing(pros::E_MOTOR_GEAR_600);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

// toggle front wings function

void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();               // Resets PID targets to 0
  chassis.drive_imu_reset();                 // Reset gyro position to 0
  chassis.drive_sensor_reset();              // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps
                                             // autonomous consistency

  ez::as::auton_selector
      .selected_auton_call(); // Calls selected auton from autonomous selector
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {

    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in
    // auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital_new_press(DIGITAL_B)) {
        autonomous();
        chassis.drive_brake_set(MOTOR_BRAKE_COAST);
      }

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        // x10
        chassis.pid_tuner_increment_p_set(1);
        chassis.pid_tuner_increment_d_set(2.5);
        chassis.pid_tuner_increment_i_set(0.01);
      } else {
        // default
        chassis.pid_tuner_increment_p_set(0.1);
        chassis.pid_tuner_increment_d_set(0.25);
        chassis.pid_tuner_increment_i_set(0.001);
      }

      chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    }

    chassis.opcontrol_tank(); // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    // intake | R2/R1 - intake/outake
    if (master.get_digital(DIGITAL_R2)) {
      intake.move_velocity(600);
    } else if (master.get_digital(DIGITAL_R1)) {
      intake.move_velocity(-600);
    } else {
      intake.move_velocity(0);
    }

    // front wings
    if (master.get_digital_new_press(DIGITAL_UP)) {
      front_wings_out = !front_wings_out;
      front_wings.set_value(front_wings_out);
    }

    // intake lifter
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      intake_lifted = !intake_lifted;
      intake_lifter.set_value(!intake_lifted);
    }

    // master.set_text(1,1,chassis.drive_get())
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}

#pragma once

#include "EZ-Template/drive/drive.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
extern pros::MotorGroup intake;
extern pros::ADIDigitalOut front_wings;


void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void combining_movements();
void interfered_example();
void defensive_side_safe();

void default_constants();
void tuning_exit_conditions();
void adjustment_conditions();
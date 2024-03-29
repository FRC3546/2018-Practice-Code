/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
 constexpr int kFrontLeftMotor = 0;
 constexpr int kFrontRightMotor = 1;
 constexpr int kRearLeftMotor = 2;
 constexpr int kRearRightMotor = 3;

 constexpr int kLeftJoystickIndex = 0;
 constexpr int kTwistRawAxis = 3;

//Grab motor = PWM port
 constexpr int kGrabMotor1 = 5;
 constexpr int kGrabMotor2 = 4;
 constexpr int kJoystickGrabButton = 3;
 constexpr int kJoystickRelease Button = 2;
// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;

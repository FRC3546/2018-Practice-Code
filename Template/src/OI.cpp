/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "RobotMap.h"

#include <WPILib.h>

OI::OI() {
	this->leftJoystick = new Joystick(kLeftJoystickIndex);
	leftJoystick.whenpressed
}


OI::~OI() {
	delete this->leftJoystick;
}

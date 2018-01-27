#include <Commands/DriveCommand.h>
#include "Robot.h"

#include "Robot.h"

DriveCommand::DriveCommand() {
}

// Called just before this Command runs the first time
void DriveCommand::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() {
	Robot::driveTrain->takeJoystickInputs(Robot::oi->leftJoystick);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveCommand::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveCommand::End() {
	Robot::driveTrain->stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveCommand::Interrupted() {
	End();
}

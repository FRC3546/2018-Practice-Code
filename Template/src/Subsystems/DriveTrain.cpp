#include "DriveTrain.h"
#include "../RobotMap.h"

#include <RobotDrive.h>

DriveTrain::DriveTrain() : Subsystem("ExampleSubsystem") {
	this->frontLeft = new VictorSP(kFrontLeftMotor);
	this->frontRight = new VictorSP(kFrontRightMotor);
	this->rearLeft = new VictorSP(kRearLeftMotor);
	this->rearRight = new VictorSP(kRearLeftMotor);
	this->driveTrain = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
}

DriveTrain::~DriveTrain() {
	delete this->driveTrain;
	delete this->frontLeft;
	delete this->frontRight;
	delete this->rearLeft;
	delete this->rearRight;
}

void DriveTrain::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void DriveTrain::takeJoystickInputs(Joystick *stick){
	 this->driveTrain->MecanumDrive_Cartesian(
			 stick->GetX(),
			 stick->GetY(),
			 stick->GetRawAxis(kTwistRawAxis),
			 0 //TODO Change to GyroSubsystem.get_angle() or similar
			 );
}

void DriveTrain::stop(){
	this->driveTrain->MecanumDrive_Cartesian(0,0,0);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

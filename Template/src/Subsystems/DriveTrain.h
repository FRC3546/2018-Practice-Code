#ifndef DriveTrain_H
#define DriveTrain_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>
#include <RobotDrive.h>
#include <Joystick.h>

using namespace frc;

class DriveTrain : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	VictorSP *frontLeft, *frontRight, *rearLeft, *rearRight;
	RobotDrive *driveTrain;

public:
	DriveTrain();
	~DriveTrain();
	void InitDefaultCommand();
	void takeJoystickInputs(Joystick*);
	void stop();
};

#endif  // DriveTrain_H

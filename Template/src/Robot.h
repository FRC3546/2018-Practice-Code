#ifndef Robot_H
#define Robot_H

#include <Commands/Command.h>
#include <Commands/DriveCommand.h>
#include <Commands/Scheduler.h>

#include <SmartDashboard/SendableChooser.h>

#include <LiveWindow/LiveWindow.h>
#include <TimedRobot.h>

#include <Commands/ExampleCommand.h>
#include <Commands/MyAutoCommand.h>


#include <OI.h>
#include <Subsystems/DriveTrain.h>


class Robot : public frc::TimedRobot {
public:
	static OI* oi;
	static DriveTrain* driveTrain;

	frc::Command* driveCommand;

	Robot();
	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc::Command* m_autonomousCommand = nullptr;
	ExampleCommand m_defaultAuto;
	MyAutoCommand m_myAuto;
	frc::SendableChooser<frc::Command*> m_chooser;
};

#endif

#ifndef Drive_H
#define Drive_H

#include <JoyStick.h>
#include <Commands/Command.h>
using namespace frc;

class DriveCommand : public Command {
public:
	DriveCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Drive_H

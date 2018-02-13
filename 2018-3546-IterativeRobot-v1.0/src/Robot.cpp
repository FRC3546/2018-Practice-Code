#include <iostream>
#include "WPILib.h"
#include "AHRS.h"
#include <AnalogPotentiometer.h>
#include <DigitalInput.h>
#include <SPI.h>
#include <LiveWindow/LiveWindow.h>
#include "ctre/Phoenix.h"

/**
 * This is a demo program showing the use of the navX MXP to implement
 * field-centric ("field-oriented") drive system control of a Mecanum-
 * based drive system.  Note that field-centric drive can also be used
 * with other Holonomic drive systems (e.g., OmniWheel, Swerve).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 */

class Robot: public SampleRobot
{

    // PWM CHANNELS FOR DRIVETRAIN MOTORS
    const static int frontLeftChannel = 0;
    const static int rearLeftChannel = 1;
    const static int frontRightChannel = 2;
    const static int rearRightChannel = 3;

    // JOYSTICK DEFINITION
    const static int joystickChannel = 0;
    const static int kJoystick_Twist_RawAxis = 2;

    // PNEUMATICS CONTROL MODULE DEFINITION
    const static int pcmid = 0;			// CAN ID
    const static int gripperRight2 = 1;
    const static int gripperLeft2 = 2;
    const static int gripperBL = 3;
    const static int platform_release = 4;
    const static int platform_extend = 5;


    // GRIPPER MOTOR DEFINITION
    const static int gripperleftmotor = 2;		// CAN ID
    const static int gripperightmotor = 1;		// CAN ID

    //Solenoids
    const static int solenoid_aport = 0;
    const static int solenoid_bport = 1;


    // SUBSYSTEM DEFINITION
    RobotDrive robotDrive;    // Robot drive system
    Joystick stick;           // Driver Joystick
    AHRS *ahrs;               // navX MXP
    Compressor *c;            // create compressor

    WPI_TalonSRX *gripperLeft;
	WPI_TalonSRX *gripperRight;

	// DOUBLE SOLENOID PORTS
    DoubleSolenoid *solenoidGR_UD;
    DoubleSolenoid *solenoidGR_OC;
    DoubleSolenoid *solenoidFL_ER;
;
public:
    Robot() :
            robotDrive(frontLeftChannel, rearLeftChannel,
                       frontRightChannel, rearRightChannel),      // initialize variables in
            stick(joystickChannel)                                // same order declared above
    {

        robotDrive.SetExpiration(0.1);

        // Invert Motors on the Robot's RIGHT side
        robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
        robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);

        c = new Compressor(pcmid);
        c->SetClosedLoopControl(true);        // turn on compressor, set to false to turn off

        gripperLeft = new WPI_TalonSRX(gripperleftmotor);
        gripperRight = new WPI_TalonSRX(gripperightmotor);

        try {
            /***********************************************************************
             * navX-MXP:
             * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
             * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             *
             * navX-Micro:
             * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
             * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
             *
             * Multiple navX-model devices on a single robot are supported.
             ************************************************************************/
            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs ) {
            LiveWindow::GetInstance()->AddSensor("IMU", "gyro", ahrs);
        }

        solenoidGR_UD = new DoubleSolenoid(1, 0, 1);
        solenoidGR_OC = new DoubleSolenoid(1, 2, 3);
        solenoidFL_ER = new DoubleSolenoid(1, 4, 5);

    }


    void StopGripperMotors(void)
    {
    	gripperLeft->Set(0);
    	gripperRight->Set(0);
    }

    void GripperIntake(void)
	{
		gripperLeft->Set(0.4);
		gripperRight->Set(-0.4);
	}

    void GripperSpitout(void)
   	{
   		gripperLeft->Set(-0.65);
   		gripperRight->Set(0.65);

   	}

    /**
     * Runs the motors with Mecanum drive.
     */
    void OperatorControl()
    {

        robotDrive.SetSafetyEnabled(false);
        while (IsOperatorControl() && IsEnabled())
        {

            bool reset_yaw_button_pressed = stick.GetRawButton(1);
            if ( reset_yaw_button_pressed ) {
                ahrs->ZeroYaw();
            }


        	bool pressStop = stick.GetRawButton(12);
	            if ( pressStop ) {
	                StopGripperMotors();
	            }


        	bool pressIntake = stick.GetRawButton(3);
           		            while ( pressIntake ) {
           		                GripperIntake();
           		             pressIntake = stick.GetRawButton(3);
           		             if(!pressIntake){
           		            	 StopGripperMotors();
           		             }
            		        //    if ( pressStop ) {
            		        //        StopGripperMotors();
            		        //        break;
            		        //    }
           		            }


        	bool pressSpitout = stick.GetRawButton(4);
           		            while ( pressSpitout ) {
           		                GripperSpitout();
           		             pressSpitout = stick.GetRawButton(4);
           		             if(!pressSpitout){
           		            	 StopGripperMotors();
           		             }
            		         //   if ( pressStop ) {
            		         //       StopGripperMotors();
            		         //       break;
            		         //   }
           		            }

           	bool gripperUp = stick.GetRawButton(5);
           					if ( gripperUp ) {
           						solenoidGR_UD->Set(DoubleSolenoid::Value::kForward);

           					}

           	bool gripperDown = stick.GetRawButton(6);
           					if ( gripperDown ) {
           					    solenoidGR_UD->Set(DoubleSolenoid::Value::kReverse);

           					}

           	bool gripperOpen = stick.GetRawButton(8);
           					if ( gripperOpen ) {
           					    solenoidGR_OC->Set(DoubleSolenoid::Value::kForward);

           					}

           	bool gripperClose = stick.GetRawButton(7);
           					if ( gripperClose ) {
           					    solenoidGR_OC->Set(DoubleSolenoid::Value::kReverse);

           					}

           	bool flipperEject = stick.GetRawButton(9);
           					if ( flipperEject ) {
           					    solenoidFL_ER->Set(DoubleSolenoid::Value::kForward);

           					}

           	bool flipperRetract = stick.GetRawButton(10);
           					if ( flipperRetract ) {
           					    solenoidFL_ER->Set(DoubleSolenoid::Value::kReverse);

           					}

       /*    	solenoid->Set(DoubleSolenoid::Value::kForward);

           	solenoidGR->Set(DoubleSolenoid::Value::kForward);
           	solenoidGL->Set(DoubleSolenoid::Value::kForward);
           	solenoidBL->Set(DoubleSolenoid::Value::kForward);
           	solenoidPL_REL->Set(DoubleSolenoid::Value::kForward);
           	solenoidPL_EXT->Set(DoubleSolenoid::Value::kForward);
*/ //Out for now - update later

            try {
                /* Use the joystick X axis for lateral movement,            */
                /* Y axis for forward movement, and Z axis for rotation.    */
                /* Use navX MXP yaw angle to define Field-centric transform */
                robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetRawAxis(kJoystick_Twist_RawAxis),ahrs->GetAngle());


            } catch (std::exception& ex ) {
                std::string err_string = "Error communicating with Drive System:  ";
                err_string += ex.what();
                DriverStation::ReportError(err_string.c_str());
            }


            Wait(0.05); // wait 50ms to avoid hogging CPU cycles
        }
    }

};

START_ROBOT_CLASS(Robot);

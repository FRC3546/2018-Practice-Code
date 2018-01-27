#include <iostream>
#include "WPILib.h"
#include "AHRS.h"
#include <AnalogPotentiometer.h>
#include <DigitalInput.h>
#include <SPI.h>

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

    // Channels for the wheels
    const static int frontLeftChannel    = 3;
    const static int rearLeftChannel    = 1;
    const static int frontRightChannel    = 2;
    const static int rearRightChannel    = 0;

    const static int joystickChannel    = 0;
    const static int kJoystick_Twist_RawAxis = 3;

    const static int pot1_port = 0;
    const static int sonar1_port = 1;
    const static int accel1_port = 2;

    const static int sampleEncoder_port1 = 0;
    const static int sampleEncoder_port2 = 1;
    const static int switchPort = 0;
    const static int solenoid_aport = 0;
    const static int solenoid_bport = 1;
    const static int pcmid = 1;




    RobotDrive robotDrive;    // Robot drive system
    Joystick stick;            // Driver Joystick
    AHRS *ahrs;             // navX MXP
    AnalogPotentiometer *pot1;
    Ultrasonic *ultra1; // creates the ultra object
    AnalogAccelerometer *accel;
    Encoder *sampleEncoder;
    DigitalInput *limitSwitch;
    DoubleSolenoid *solenoid;


    Compressor *c;        // create compressor

;
public:
    Robot() :
            robotDrive(frontLeftChannel, rearLeftChannel,
                       frontRightChannel, rearRightChannel),    // initialize variables in
            stick(joystickChannel)                                // same order declared above
    {

        robotDrive.SetExpiration(0.1);
        robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);    // invert left side motors
        robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);    // change to match your robot

        pot1 = new AnalogPotentiometer(pot1_port, 360, 0);
        ultra1 = new Ultrasonic(sonar1_port, sonar1_port);
        accel = new AnalogAccelerometer(accel1_port); //create accelerometer on analog input 0
        sampleEncoder = new Encoder(sampleEncoder_port1, sampleEncoder_port2, false, Encoder::EncodingType::k4X);
            sampleEncoder->Reset();
        limitSwitch = new DigitalInput(switchPort);
        solenoid = new DoubleSolenoid(pcmid, solenoid_aport, solenoid_bport);






        c = new Compressor(pcmid);
        c->SetClosedLoopControl(true);        // turn on compressor, set to false to turn off

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
            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }


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

            printf("Potentiometer: %f\n", pot1->Get());
            printf("RadarSensor: %f\n", ultra1->GetRangeInches());
            printf("Accel: %f\n", accel->GetAcceleration());
            printf("sampleEncoder: %d\n" , sampleEncoder -> GetRaw ());
            // Mijan's code is better than mine.
            if(limitSwitch->Get())
                printf("Switch = true\n");
            else
                printf("Switch = false\n");

            solenoid->Set(DoubleSolenoid::Value::kForward);


            Wait(0.05); // wait 50ms to avoid hogging CPU cycles
        }
    }

};

START_ROBOT_CLASS(Robot);
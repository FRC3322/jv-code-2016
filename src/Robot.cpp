#include "WPILib.h"
#include "AHRS.h"


class Robot: public IterativeRobot
{
	std::shared_ptr<NetworkTable> table;
	Talon claw;
	RobotDrive myRobot;
	Joystick stick;
    AHRS *ahrs;
	LiveWindow *lw;
	int autoLoopCounter;

public:
	Robot() :
        table(NULL),
		claw(4),
		myRobot(2, 3, 0, 1),
		stick(0),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		autoLoopCounter(0)
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetMaxOutput(.5);
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}
private:
	void RobotInit()
	{
		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if ( ahrs ) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}
	}
	void AutonomousInit()
	{
		autoLoopCounter = 0;
		ahrs->Reset();
	}

	void AutonomousPeriodic()
	{
        SmartDashboard::PutNumber(  "IMU_TotalYaw(ours)",         ahrs->GetAngle());
		if(1) {
			float angleError = ahrs->GetAngle();
			if (angleError > 180) {
				angleError = angleError-360;
			}
			angleError = angleError*.01;
			//myRobot.SetLeftRightMotorOutputs(angleError, -angleError);
			myRobot.Drive(-.5, -angleError);
			autoLoopCounter++;
		} else {
			myRobot.Drive(0.0, 0.0);
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		if (stick.GetRawButton(5)) {
			claw.Set(1, 0);
		} else if (stick.GetRawButton(6)) {
			claw.Set(-1, 0);
		} else {
			claw.Disable();
		}
		//myRobot.TankDrive(stick.GetRawAxis(1),stick.GetRawAxis(5));
		myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)

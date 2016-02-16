#include "WPILib.h"
#include "AHRS.h"

const double liftSetPointUp = 4.0;
const double liftSetPointDown = 1.0;
const int ticksPerSecond = 50;

class Robot: public IterativeRobot
{
	AnalogInput liftPot;
	PIDController liftController;
	AnalogInput ultrasonic;
	Encoder driveEncoder;
	std::shared_ptr<NetworkTable> table;
	Talon shooter;
	Talon lift;
	RobotDrive myRobot;
	Joystick driveStick;
	Joystick operatorStick;
    AHRS *ahrs;
	LiveWindow *lw;
	bool liftLastButton;
	double liftSetPoint;
	int autonState;
	int autonTimer;


public:
	Robot() :
		liftPot(0),
		liftController(.2, 0.0, 0.0, &liftPot, &lift),
		ultrasonic(1),
		driveEncoder(0, 1, false, Encoder::k4X),
        table(NULL),
		shooter(5),
		lift(4),
		myRobot(2, 3, 0, 1),
		driveStick(0),
		operatorStick(1),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		liftLastButton(false),
		liftSetPoint(liftSetPointUp),
		autonState(0),
		autonTimer(0)
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetMaxOutput(.5);
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		driveEncoder.SetReverseDirection(true);
		driveEncoder.SetSamplesToAverage(5);
		// driveEncoder.SetDistancePerPulse(1.0 / 360.0 * 2.0 * 3.1415 * 1.5);
		driveEncoder.SetDistancePerPulse(1.0 / 360.0);
		driveEncoder.SetMinRate(1.0);

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
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if ( ahrs ) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}
	void AutonomousInit()
	{
		autonState = 0;
		autonTimer = 0;
		ahrs->Reset();
		driveEncoder.Reset();

		// start controlling the lift
		liftSetPoint = liftSetPointUp;
		liftController.SetSetpoint(liftSetPoint);
		liftController.Enable();
	}

	void AutonomousPeriodic()
	{
		autonTimer++;
        SmartDash();

        if (autonState == 0) {
			// drive forward for a specified distance
			DriveGyro (-0.5, 0);

			// if (driveEncoder.GetDistance() > 3) {
			if (autonTimer > 5 * ticksPerSecond) {
				myRobot.Drive (0,0);
				autonState++;
			}
		}

		if (autonState == 1) {
		  //turn to a specified angle
			myRobot.Drive (-.5, 1);
			if (GetAngle() > 90) {
				myRobot.Drive (0,0);
				driveEncoder.Reset();
				autonTimer = 0;
				autonState++;
			}
		}

        if (autonState == 2) {
			// drive forward for a specified distance
        	DriveGyro(-0.5, 90);
			 // if (driveEncoder.GetDistance() > 10) {
        	if (autonTimer > 3 * ticksPerSecond) {
				myRobot.Drive (0,0);
				autonState++;
			 }
        }
        if (autonState == 3) {
        	shooter.Set(1.0, 0);
        }
	}

	void TeleopInit()
	{
		liftLastButton = false;
	}

	void TeleopPeriodic()
	{
		SmartDash();

		// Control the lift arm motor
		if (operatorStick.GetRawButton(1)) {
			lift.Set(0.3, 0);
		} else if (operatorStick.GetRawButton(2)) {
			lift.Set(-0.3, 0);
		} else {
			lift.Disable();
		}
		/*
		if (!liftLastButton && operatorStick.GetRawButton(1)) {
			if (liftSetPoint == liftSetPointUp) {
				liftSetPoint = liftSetPointDown;
			} else {
				liftSetPoint = liftSetPointUp;
			}
			liftController.SetSetpoint(liftSetPoint);
		}
		liftLastButton = operatorStick.GetRawButton(1);
		*/

		// Control the shooter
		if (operatorStick.GetRawButton(5)) {
			shooter.Set(0.5, 0);
		} else if (operatorStick.GetRawButton(6)) {
			shooter.Set(-0.5, 0);
		} else {
			shooter.Disable();
		}

		//myRobot.TankDrive(driveStick.GetRawAxis(1),driveStick.GetRawAxis(5));
		myRobot.ArcadeDrive(driveStick); // drive with arcade style (use right stick)
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void DriveGyro(double outputMagnitude, double angle)
	{
		float angleError = GetAngle() - angle;
		angleError = angleError*.01;
		myRobot.Drive(outputMagnitude, -angleError);
	}
	void SmartDash(){

		SmartDashboard::PutNumber("IMU Yaw", ahrs->GetAngle());
		SmartDashboard::PutNumber("Encoder Distance", -driveEncoder.GetDistance());
		SmartDashboard::PutNumber("Encoder Rate", driveEncoder.GetRate());
		SmartDashboard::PutNumber("Lift Potentiometer Voltage", liftPot.GetVoltage());
		SmartDashboard::PutNumber("Ultrasonic", ultrasonic.GetValue());
	}

	float GetAngle()
	{
		float angle = ahrs->GetAngle();
		if (angle > 180) {
			angle = angle-360;
		}
		return(angle);
	}

};

START_ROBOT_CLASS(Robot)

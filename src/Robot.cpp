#include "WPILib.h"
#include "AHRS.h"

const double liftSetPointUp = 4.0;
const double liftSetPointDown = 1.0;

class Robot: public IterativeRobot
{
	AnalogInput liftPot;
	PIDController liftController;
	AnalogInput ultrasonic;
	Encoder driveEncoder;
	std::shared_ptr<NetworkTable> table;
	Talon collector;
	Talon lift;
	RobotDrive myRobot;
	Joystick stick;
    AHRS *ahrs;
	LiveWindow *lw;
	bool liftLastButton;
	double liftSetPoint;
	int autonState;


public:
	Robot() :
		liftPot(0),
		liftController(.2, 0.0, 0.0, &liftPot, &lift),
		ultrasonic(1),
		driveEncoder(0, 1, false, Encoder::k4X),
        table(NULL),
		collector(5),
		lift(4),
		myRobot(2, 3, 0, 1),
		stick(0),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		liftLastButton(false),
		liftSetPoint(liftSetPointUp),
		autonState(0)
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetMaxOutput(.5);
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
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
		ahrs->Reset();
		driveEncoder.Reset();

		// start controlling the lift
		liftSetPoint = liftSetPointUp;
		liftController.SetSetpoint(liftSetPoint);
		liftController.Enable();
	}

	void AutonomousPeriodic()
	{
        SmartDash();
		/*
		float angleError = ahrs->GetAngle();
		if (angleError > 180) {
			angleError = angleError-360;
		}
		angleError = angleError*.01;
		//myRobot.SetLeftRightMotorOutputs(angleError, -angleError);
		myRobot.Drive(-.5, -angleError);
		*/

        if (autonState == 0){
			// drive forward for a specified distance
			DriveGyro (0.5, 0);
			 if (driveEncoder.GetDistance() > 5 && ultrasonic.GetVoltage() < 2) {
				autonState++;
			}
		}

		if (autonState == 1){
			/// turn to a specified angle
			myRobot.Drive(1.0, 0.5);
			if (ahrs->GetAngle() > 90) {
				autonState++;
				driveEncoder.Reset();
			}
		}

        if (autonState == 2){
			// drive forward for a specified distance
			DriveGyro (0.5, 90);
			 if (driveEncoder.GetDistance() > 5) {
				autonState++;
				myRobot.Drive (0,0);
			 }
        }
        if (autonState ==3){
        	collector.Set(1.0, 0);
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
		if (!liftLastButton && stick.GetRawButton(4)) {
			if (liftSetPoint == liftSetPointUp) {
				liftSetPoint = liftSetPointDown;
			} else {
				liftSetPoint = liftSetPointUp;
			}
			liftController.SetSetpoint(liftSetPoint);

		}
		liftLastButton = stick.GetRawButton(4);

		// Control the collector
		if (stick.GetRawButton(5)) {
			collector.Set(1.0, 0);
		} else if (stick.GetRawButton(6)) {
			collector.Set(-1.0, 0);
		} else {
			collector.Disable();
		}

		// Drive
		//myRobot.TankDrive(stick.GetRawAxis(1),stick.GetRawAxis(5));
		myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void DriveGyro(double outputMagnitude, double angle)
	{
		float angleError = ahrs->GetAngle()-angle;
		if (angleError > 180) {
			angleError = angleError-360;
		}
		angleError = angleError*.01;
		//myRobot.SetLeftRightMotorOutputs(angleError, -angleError);
		myRobot.Drive(outputMagnitude, -angleError);
	}
	void SmartDash(){

		SmartDashboard::PutNumber("IMU Yaw", ahrs->GetAngle());
		SmartDashboard::PutNumber("Encoder Distance", driveEncoder.GetDistance());
		SmartDashboard::PutNumber("Encoder Rate", driveEncoder.GetRate());
		SmartDashboard::PutNumber("Lift Potentiometer Voltage", liftPot.GetVoltage());
		SmartDashboard::PutNumber("Ultrasonic", ultrasonic.GetVoltage());
	}
};

START_ROBOT_CLASS(Robot)

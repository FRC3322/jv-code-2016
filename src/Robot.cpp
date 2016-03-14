#include "WPILib.h"
#include "AHRS.h"

const double liftSetPointUp = 4.0;
const double liftSetPointDown = 1.0;
const double ticksPerSecond = 44.0;
const double forwardDrive = 4.0;
const double afterTurnForwardDrive = 2.0;
const double turnAngle = 90.0;
const double liftDown = 4.0;
const double liftBack = 2.75;
const double liftVertical = 3.5;

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
	Joystick techStick;
    AHRS *ahrs;
	LiveWindow *lw;
	double multiplier;
	int autonState;
	int autonTimer;


public:
	Robot() :
		liftPot(0),
		liftController(-0.5, 0.0, 0.0, &liftPot, &lift),
		ultrasonic(1),
		driveEncoder(0, 1, false, Encoder::k4X),
        table(NULL),
		shooter(5),
		lift(4),
		myRobot(2,3,0,1),
		driveStick(0),
		techStick(1),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		multiplier(0),
		autonState(0),
		autonTimer(0)
	{
		myRobot.SetExpiration(0.1);
		myRobot.SetMaxOutput(1);
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
		shooter.StopMotor();
		lift.StopMotor();

		// start controlling the lift
		// liftSetPoint = liftSetPointUp;
		// liftController.SetSetpoint(liftSetPoint);
		// liftController.Enable();
	}
	void AutonomousPeriodic()
	{
		SmartDash();
		autonTimer++;

		// lower lift
		if (autonState == 0) {
			lift.Set(-0.5, 0);

			if (autonTimer > 5 * ticksPerSecond) {
				lift.Set(0, 0);
				autonState++;
			}
		}

		// drive forward
		if (autonState == 1) {
			DriveGyro(-.5, 0);

			if (driveEncoder.GetDistance() > forwardDrive) {
				DriveGyro(0, 0);
				autonState++;
			}
		}

		// turn right/left 90 degrees
		if (autonState == 2) {
			myRobot.SetLeftRightMotorOutputs(-0.5, 0.5); //Adjust on spot

			if (abs(GetAngle()) > abs(turnAngle)) {
				DriveGyro(0, turnAngle);
				driveEncoder.Reset();
				autonState++;
			}
		}

		// drive forward after turning
		if (autonState == 3) {
			DriveGyro (-0.5, turnAngle);

	        if (driveEncoder.GetDistance() > afterTurnForwardDrive) {
	        	DriveGyro (0,turnAngle);
	            autonState++;
	        }

		}

		// shoot
		if (autonState == 4) {
			shooter.Set (1,0);
		}
	}
	void TeleopInit()
	{
		multiplier=.75;
		lift.Disable();
		shooter.Disable();
	}
	void TeleopPeriodic()
	{
		SmartDash();

		// Control the lift arm motor

		if (techStick.GetRawButton(1)) {
			liftController.Enable();
			liftController.SetSetpoint(liftDown);
		}

		if (techStick.GetRawButton(2)) {
			liftController.Enable();
			liftController.SetSetpoint(liftVertical);
		}

		if (techStick.GetRawButton(3)) {
			liftController.Enable();
			liftController.SetSetpoint(liftBack);
		}

		if (techStick.GetRawButton(10)) {
			liftController.Disable();
		}

		// Control the shooter
		if (techStick.GetRawAxis(2)) {
			shooter.Set(1, 0);
		} else if (techStick.GetRawAxis(3)) {
			shooter.Set(-1, 0);
		} else if (techStick.GetRawButton(8)) {
			shooter.Disable();
		} else {
			shooter.Set(0.1, 0);
		}

		if (techStick.GetRawButton(5)) {
			liftController.Disable();
			lift.Set(0.5,0);
		}

		if (techStick.GetRawButton(6)) {
			lift.Set(-0.5,0);
			liftController.Disable();
		}

		if(driveStick.GetRawButton(1)){
			multiplier = 1;
		}

		if(driveStick.GetRawButton(3)){
			multiplier=.75;
		}

		//myRobot.TankDrive(multiplier*driveStick.GetRawAxis(1),multiplier*driveStick.GetRawAxis(5));
		myRobot.ArcadeDrive(multiplier*driveStick.GetY(),multiplier*driveStick.GetX()); // drive with arcade style (use right stick)
	}
	void TestPeriodic()
	{
		lw->Run();
	}

	void DriveGyro(double outputMagnitude, double angle)
	{
		float angleError = GetAngle() - angle;
		angleError = angleError*.02;
		myRobot.Drive(outputMagnitude, -angleError);
	}
	void SmartDash(){

		SmartDashboard::PutNumber("IMU Yaw", GetAngle());
		SmartDashboard::PutNumber("Encoder Distance", driveEncoder.GetDistance());
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

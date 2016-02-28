#include "WPILib.h"
#include "AHRS.h"

const double liftSetPointUp = 4.0;
const double liftSetPointDown = 1.0;
const int ticksPerSecond = 44;
const double forwardDrive = 4.0;
const double turnDistance = 2.0;
const double turnAngle = 90.0;

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
		myRobot(2,3,0,1),
		driveStick(0),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		liftLastButton(false),
		liftSetPoint(liftSetPointUp),
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

	        if (driveEncoder.GetDistance() > turnDistance) {
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
		liftLastButton = false;
	}

	void TeleopPeriodic()
	{
		SmartDash();

		// Control the lift arm motor
		if (driveStick.GetRawButton(1)) {
			if (liftPot.GetVoltage() > 1.1) {
				lift.Set (-1,0);
			}
			if (liftPot.GetVoltage() < 0.9) {
				lift.Set (1,0);
			}
			if ((liftPot.GetVoltage() > 0.9) && liftPot.GetVoltage() < 1.1) {
				lift.Disable();
			}
		}

		if (driveStick.GetRawButton(2)) {
			if (liftPot.GetVoltage() > 2.1) {
				lift.Set (-1,0);
			}
			if (liftPot.GetVoltage() < 1.9) {
				lift.Set (1,0);
			}
			if ((liftPot.GetVoltage() > 1.9) && liftPot.GetVoltage() < 2.1) {
				lift.Disable();
			}
		}

		if (driveStick.GetRawButton(4)) {
			if (liftPot.GetVoltage() > 3.1) {
				lift.Set (-1,0);
			}
			if (liftPot.GetVoltage() < 2.9) {
				lift.Set (1,0);
			}
			if ((liftPot.GetVoltage() > 2.9) && liftPot.GetVoltage() < 3.1) {
				lift.Disable();
			}
		}

		if (driveStick.GetRawButton(3)) {
			if (liftPot.GetVoltage() > 4.1) {
				lift.Set (-1,0);
			}
			if (liftPot.GetVoltage() < 3.9) {
				lift.Set (1,0);
			}
			if ((liftPot.GetVoltage() > 3.9) && liftPot.GetVoltage() < 4.1) {
				lift.Disable();
			}
		}

		// Control the shooter
		if (driveStick.GetRawAxis(2)) {
			shooter.Set(1, 0);
		} else if (driveStick.GetRawAxis(3)) {
			shooter.Set(-1, 0);
		} else {
			shooter.StopMotor();
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


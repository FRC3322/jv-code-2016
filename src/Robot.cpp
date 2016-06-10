#include "WPILib.h"
#include "AHRS.h"

const double liftSetPointUp = 4.0;
const double liftSetPointDown = 1.0;
const double ticksPerSecond = 44.0;
const double forwardDrive = 4.0;
const double afterTurnForwardDrive = 2.0;
const double turnAngle = 90.0;
const double liftForward = 4.31;
const double liftBack = 1.55;
const double liftVertical = 2.62;

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
	bool driveFast;
	int autonState;
	int timer;
	bool shooterDefaultOn;
	bool lastShooterToggleButton;
	bool driveToggleButton;

public:
	Robot() :
		liftPot(0),
		liftController(0.05, 0.005, 0.05, &liftPot, &lift),
		ultrasonic(1),
		driveEncoder(0, 1, false, Encoder::k4X),
        table(NULL),
		shooter(5),
		lift(4),
		myRobot(2,3,0,1),
		driveStick(1),
		techStick(0),
        ahrs(NULL),
		lw(LiveWindow::GetInstance()),
		driveFast(false),
		autonState(0),
		timer(0),
		shooterDefaultOn(true),
		lastShooterToggleButton(false),
		driveToggleButton(false)
	{
		myRobot.SetExpiration(0.1);
		// myRobot.SetMaxOutput(driveFast ? 0.6 : 0.3);
		//myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
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
		timer = 0;
		ahrs->Reset();
		driveEncoder.Reset();
		shooter.Disable();
		lift.Disable();
	}
	void AutonomousPeriodic()
	{
		SmartDash(0, 0);
		timer++;

		// lower lift
		if (autonState == 0) {
			lift.Set(-0.5, 0);

			if (timer > 5 * ticksPerSecond) {
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
		timer = 0;
		shooterDefaultOn=true;
		lift.Disable();
		liftController.Disable();
		shooter.Disable();
	}

	void TeleopPeriodic()
	{
		double liftAngle;

		// compute average of several liftPot samples
		double liftPotTotal;
		int liftPotCount;
		for (liftPotCount = 0; liftPotCount < 50; liftPotCount++) {
			liftPotTotal += liftPot.GetVoltage();
		}
		double liftPotAverage = liftPotTotal / liftPotCount;
		liftAngle = (liftPotAverage-liftForward)*3.1415/2.0/(liftVertical-liftForward);
		// liftAngle = 45;
		lift.Set(-0.3 * cos(liftAngle));

/*
		// Control the arm automatically
		if (techStick.GetRawButton(2)) { // B
			liftController.Disable();
			liftController.Enable();
			liftController.SetSetpoint(liftForward);
		}
		if (techStick.GetRawButton(4)) { // Y
			liftController.Disable();
			liftController.Enable();
			liftController.SetSetpoint(liftVertical);
		}
		if (techStick.GetRawButton(3)) { // X
			liftController.Disable();
			liftController.Enable();
			liftController.SetSetpoint(liftBack);
		}
		if (techStick.GetRawButton(10)) { // push down right stick
			liftController.Disable();
		}

		// Control the shooter
		if (techStick.GetRawAxis(2)) {
			shooter.Set(1, 0);
		} else if (techStick.GetRawAxis(3)) {
			shooter.Set(-1, 0);
		} else if (shooterDefaultOn) {
			shooter.Set(0.1, 0);
		} else {
			shooter.Disable();
		}

		// Control the default state of the shooter
		if (!lastShooterToggleButton && techStick.GetRawButton(8)) {
			shooterDefaultOn = !shooterDefaultOn;
		}
		lastShooterToggleButton = techStick.GetRawButton(8);
*/
/*
		// Control the arm manually
		if (techStick.GetRawButton(5)) { // LB
			liftController.Disable();
			lift.Set(0.5,0); // down
		}
		if (techStick.GetRawButton(6)) { // RB
			liftController.Disable();
			lift.Set(-0.5,0); // up
		}
*/

		// Control the drive speed
		if (!driveToggleButton && driveStick.GetRawButton(1)) { // A
			driveFast = !driveFast;
			// myRobot.SetMaxOutput(driveFast ? 0.6 : 0.3);
		}
		driveToggleButton = driveStick.GetRawButton(1);

		// Compute power for the drive motors (do this manually so we can adjust for different strength motors)
		double basePower = (driveFast ? 0.75 : 0.3) * (-driveStick.GetRawAxis(1));
		double turn = (driveFast ? 0.75 : 0.75) * driveStick.GetRawAxis(4);
		double leftPower = (basePower + turn); // left motor is stronger?
		leftPower *= (basePower > 0) ? 0.9 : 1.03;
		double rightPower = basePower - turn;
		myRobot.SetLeftRightMotorOutputs(leftPower, rightPower);

		//myRobot.ArcadeDrive(driveStick.GetY(), driveStick.GetX());
		//myRobot.TankDrive(driveStick.GetRawAxis(1),driveStick.GetRawAxis(5));
		//myRobot.ArcadeDrive(driveStick); // drive with arcade style (use right stick)

		SmartDash(liftPotAverage, liftAngle);
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
	void SmartDash(double liftPotAverage, double liftAngle)
	{
		SmartDashboard::PutNumber("IMU Yaw", GetAngle());
		SmartDashboard::PutNumber("Encoder Distance", driveEncoder.GetDistance());
		SmartDashboard::PutNumber("Encoder Rate", driveEncoder.GetRate());
		SmartDashboard::PutNumber("Lift Potentiometer Voltage (average)", liftPotAverage);
		SmartDashboard::PutNumber("Lift Angle", liftAngle);
		SmartDashboard::PutNumber("Lift Power", -0.3 * cos(liftAngle));
		SmartDashboard::PutNumber("Cos(Pi)", cos(3.1415));


		SmartDashboard::PutNumber("Ultrasonic", ultrasonic.GetValue());
		SmartDashboard::PutNumber("Speed", driveStick.GetRawAxis(1));
		SmartDashboard::PutNumber("Turn", driveStick.GetRawAxis(4));
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

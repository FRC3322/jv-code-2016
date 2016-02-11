#include "WPILib.h"
#include "AHRS.h"


class Robot: public IterativeRobot
{
	AnalogInput potentiometer;
	Encoder m_encoder;
	std::shared_ptr<NetworkTable> table;
	Talon claw;
	RobotDrive myRobot;
	Joystick stick;
    AHRS *ahrs;
	LiveWindow *lw;
	int autoLoopCounter;

public:
	Robot() :
		potentiometer(2),
		m_encoder(0, 1, false, Encoder::k4X),
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
		m_encoder.SetSamplesToAverage(5);
		// m_encoder.SetDistancePerPulse(1.0 / 360.0 * 2.0 * 3.1415 * 1.5);
		m_encoder.SetDistancePerPulse(1.0 / 360.0);
		m_encoder.SetMinRate(1.0);

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
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}
	void AutonomousInit()
	{
		autoLoopCounter = 0;
		ahrs->Reset();
	}

	void AutonomousPeriodic()
	{
        SmartDashboard::PutNumber(  "IMU_TotalYaw(ours)",         ahrs->GetAngle());
        SmartDashboard::PutNumber("Encoder Distance", m_encoder.GetDistance());
        SmartDashboard::PutNumber("Encoder Rate", m_encoder.GetRate());
        SmartDashboard::PutNumber("Potentiometer Value", potentiometer.GetValue());
        SmartDashboard::PutNumber("Potentiometer Voltage", potentiometer.GetVoltage());

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

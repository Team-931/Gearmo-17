//G17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17AutoG17Auto
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Drive/DifferentialDrive.h>
//#define usingtank
const double ramptime=0.5;
const double motorvoltage=11.5;//minimum accepted motor voltage for voltage compensation

class Robot: public frc::IterativeRobot {
	bool tankdrive=false;
	//::WPI_TalonSRX t_motor{9};
	::WPI_TalonSRX m_frontleft{1};
	::WPI_TalonSRX m_backleft{12};
	::WPI_TalonSRX m_frontright{11};
	::WPI_TalonSRX m_backright{7};
	frc::SpeedControllerGroup m_left{ m_frontleft, m_backleft};
	frc::SpeedControllerGroup m_right{ m_frontright, m_backright};
	frc::DifferentialDrive ImperialShuttle {m_left,m_right };// robot drive system
    frc::DigitalInput LowerLimit{0}, UpperLimit{1};//Part of test
	::PigeonIMU position{&m_backright};//This sensor is attached to the controller -- it must initialize after it.
	frc::Timer m_time;
	std::string fieldData; // stores the value from GetGameSpecificMessage
		bool center{true};//--
		bool left{true};//-----these can be used  to tell the robot what side it is on
		bool right{true};//---
		double speed;

public:
	Robot() {
		ImperialShuttle.SetExpiration(0.1);
	}
void RobotInit(){
	m_frontleft.ConfigOpenloopRamp(ramptime,0);
	m_frontright.ConfigOpenloopRamp(ramptime,0);
	m_backleft.ConfigOpenloopRamp(ramptime,0);
	m_backright.ConfigOpenloopRamp(ramptime,0);
	m_frontleft.ConfigVoltageCompSaturation(motorvoltage,0);
	m_frontright.ConfigVoltageCompSaturation(motorvoltage,0);
	m_backleft.ConfigVoltageCompSaturation(motorvoltage,0);
	m_backright.ConfigVoltageCompSaturation(motorvoltage,0);
	frc::SmartDashboard::SetDefaultString("Field data: ", "LLL");
	frc::SmartDashboard::SetDefaultBoolean("Left", false);
	frc::SmartDashboard::SetDefaultBoolean("Right", false);
	frc::SmartDashboard::SetDefaultBoolean("Center", false);

	}
void AutonomousInit(){
		m_time.Reset();
// the field system will give us this in a match.
		fieldData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
// outside a match read it off the dashboard.
		if(fieldData[0] == 0) fieldData = frc::SmartDashboard::GetString("Field data: ", "LLL");
		else frc::SmartDashboard::PutString("Field data: ", fieldData);
// temporary: test field data. Todo: use value to choose a path. 'L' means switch is on left, "R" ...
		frc::SmartDashboard::PutString("Switch is ", fieldData.substr(0,1));

		// these make sure the battery status doesn't affect speed
		m_frontleft.EnableVoltageCompensation(true);
		m_frontright.EnableVoltageCompensation(true);
		m_backleft.EnableVoltageCompensation(true);
		m_backright.EnableVoltageCompensation(true);
		//game starting positions
		center = ::SmartDashboard::GetBoolean("Center", true);
		left = ::SmartDashboard::GetBoolean("Left", true);
		right = ::SmartDashboard::GetBoolean("Right", true);
	}//End Autonomous init
	double startingangle=position.GetFusedHeading();;//angle the robot initializes with. allows for the robot to be placed at any angle
	double turnangle;
	double Newstartingangle; //angle after turn
	int turningangle=90;//angle of turn
	void turnLeft(double timevariable, double robotspeed){
		::SmartDashboard::PutString("left", "left");
		m_time.Start();
		if (m_time.Get()<=timevariable){
			 Newstartingangle=startingangle-turningangle;}
		else robotspeed=0;
		ImperialShuttle.CurvatureDrive(robotspeed, 0.1*((Newstartingangle)-turnangle), false);
		if(m_time.Get()>=timevariable)
			m_time.Stop();
			m_time.Reset();}
	void turnRight(double timevariable, double robotspeed){
		::SmartDashboard::PutString("right", "right");
			m_time.Start();
			if (m_time.Get()<=timevariable){
				 Newstartingangle=startingangle+turningangle;}
			else robotspeed=0;
			ImperialShuttle.CurvatureDrive(robotspeed, 0.01*(Newstartingangle-turnangle), false);
			if(m_time.Get()>=timevariable)
				m_time.Stop();
				m_time.Reset();}
	void straight(double timevariable, double robotspeed){
		::SmartDashboard::PutString("straight", "straight");
					m_time.Start();
					if (m_time.Get()<=timevariable){
						 Newstartingangle=startingangle;}
					else robotspeed=0;
					ImperialShuttle.CurvatureDrive(robotspeed, 0.01*(Newstartingangle-turnangle), false);
					if(m_time.Get()>=timevariable)
						m_time.Stop();
						m_time.Reset();}
	void AutonomousPeriodic(){
//-----------------------------drive------------------------------------//
		if 	(fieldData[0]=='L'){
		straight(1,0.5);}
		else if(fieldData[0]=='R'){
			straight(1,0.25);
			turnLeft(1,0.25);
		}
	}
};


START_ROBOT_CLASS(Robot)

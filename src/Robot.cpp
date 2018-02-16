//G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Drive/DifferentialDrive.h>
//#define usingtank
const double ramptime=0.5;
const double motorvoltage=11.5;//minimum accepted motor voltage for voltage compensation
class wing{
	frc::Solenoid lift1,
				  lift2;
	frc::Servo drop;
	frc::Timer m_freezetime;
bool isrampreleased;
public:
	wing(int module_number):lift1(module_number, 0),
							lift2(module_number, 1),
							drop(module_number/*side one or side 2*/),
							isrampreleased(false){}
	void KeepExtended(){
		drop.Set(1);}
	void ReleaseWing(){
		isrampreleased=true;
		drop.Set(0);
		m_freezetime.Start();}
	void RaiseRamp(){
		if(isrampreleased==true){
		lift1.Set(true);
		//lift2.Set(true);
		}}
	void checkReleaseTimer(){

			m_freezetime.Stop();
			m_freezetime.Reset();

		}
};
class Robot: public frc::IterativeRobot {
	bool tankdrive=false;
	//::WPI_TalonSRX t_motor{9};
	::WPI_TalonSRX m_frontleft{1};
	::WPI_TalonSRX m_backleft{12};
	::WPI_TalonSRX m_frontright{11};
	::WPI_TalonSRX m_backright{7};
	::WPI_TalonSRX grip1{10};
	::WPI_TalonSRX grip2{6};
	::WPI_TalonSRX elev{5};
	frc::SpeedControllerGroup grip{ grip1,grip2};
	frc::SpeedControllerGroup m_left{ m_frontleft, m_backleft};
	frc::SpeedControllerGroup m_right{ m_frontright, m_backright};
	frc::DifferentialDrive myRobot { m_left,m_right };// robot drive system
    frc::DoubleSolenoid gripangle{0,4,5};//todo compare to reality
    ::wing leftwing{0};
    ::wing rightwing{1};
	::PigeonIMU position{&m_backright};//This sensor is attached to the controller -- it must initialize after it.
	frc::Joystick stick { 0 };  // driver stick
	frc::Joystick Ostick{ 1 };  // operator stick
	frc::Timer m_time;

	std::string fieldData; // stores the value from GetGameSpecificMessage
	bool doTurn;    //temporary: will the autonomous routine turn?
	bool blue{true};
		bool center{true};//--
		bool left{true};//-----these can be used  to tell the robot what side it is on
		bool right{true};//---
public:
	Robot() {
		myRobot.SetExpiration(0.1);
	}
	void RobotInit(){
		grip1.SetInverted(true);
		elev.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
		//printf("Entering RobotInit\n");
		leftwing.KeepExtended();
		rightwing.KeepExtended();
	m_frontleft.ConfigOpenloopRamp(ramptime,0);
	m_frontright.ConfigOpenloopRamp(ramptime,0);
	m_backleft.ConfigOpenloopRamp(ramptime,0);
	m_backright.ConfigOpenloopRamp(ramptime,0);
		//printf("Old code run\n");
	m_frontleft.ConfigVoltageCompSaturation(motorvoltage,0);
	m_frontright.ConfigVoltageCompSaturation(motorvoltage,0);
	m_backleft.ConfigVoltageCompSaturation(motorvoltage,0);
	m_backright.ConfigVoltageCompSaturation(motorvoltage,0);
		//printf("New code run\n");

	// These dashboard values allow control of autonomous.
	// If it's not on the dashboard, put it there.
	frc::SmartDashboard::SetDefaultString("Field data: ", "LLL");
	frc::SmartDashboard::SetDefaultBoolean("Do autonomous turn: ", false);
	}

	void AutonomousInit(){
		m_time.Reset();
		m_time.Start();


		startingangle=position.GetFusedHeading();
// the field system will give us this in a match.
		fieldData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
// outside a match read it off the dashboard.
		if(fieldData[0] == 0) fieldData = frc::SmartDashboard::GetString("Field data: ", "LLL");
		else frc::SmartDashboard::PutString("Field data: ", fieldData);
// temporary: test field data. Todo: use value to choose a path. 'L' means switch is on left, "R" ...
		frc::SmartDashboard::PutString("Switch is ", fieldData.substr(0,1));
// read whether to follow the straight run with a turn.
		doTurn = frc::SmartDashboard::GetBoolean("Do autonomous turn: ", false);
		// these make sure the battery status doesn't affect speed
		m_frontleft.EnableVoltageCompensation(true);
		m_frontright.EnableVoltageCompensation(true);
		m_backleft.EnableVoltageCompensation(true);
		m_backright.EnableVoltageCompensation(true);
		blue = ::SmartDashboard::GetBoolean("Blue", true);
				center = ::SmartDashboard::GetBoolean("Center", true);
				left = ::SmartDashboard::GetBoolean("Left", true);
				right = ::SmartDashboard::GetBoolean("Right", true);

	}
	double startingangle;//angle the robot initializes with. allows for the robot to be placed at any angle
	double turnangle;
	double Newstartingangle; //angle after turn
	double turningangle=90;//angle of turn
	double speed;
	void turnLeft(double timevariable){
		if (m_time.Get()<=timevariable)
			 Newstartingangle=startingangle-turningangle;
	}
	void turnRight(double timevariable){
		if (m_time.Get()<=timevariable)
		 Newstartingangle=startingangle+turningangle;

	}
	void straight(double timevariable){
		if (m_time.Get()<=timevariable)
		Newstartingangle=startingangle;
	}
	void shoot(){
		grip.Set(1);//EDITABLE
	}
	void pull(){
		grip.Set(-.25);
	}
	void raiseElev(double height){
		const double diameter=1;
		static double rotation=elev.GetSelectedSensorPosition(0);
		static double circumference/*in inches*/=3.141592653585898*(diameter+((rotation/3)*3/16));// doubles only hold 15 bits
		elev.Set(ControlMode::Position,height/circumference);//EDITABLE
	}
	void PositionIsRight(){
		if (fieldData[0]=='L'){
			straight(0.75);
			turnLeft(1);
			straight(3);
			turnRight(4);
			straight(5);
		}
		else{
			straight(3);
			turnLeft(0.5);
		}
	}
void PositionisLeft(){
	if (fieldData[0]=='R'){
				straight(0.75);
				turnLeft(1);
				straight(3);
				turnRight(4);
				straight(5);
			}
			else{
				straight(3);
				turnRight(0.5);
			}
}
void PositionisCenter(){
	if (fieldData[0]=='R'){
					straight(0.75);
					turnRight(1);
					straight(2);
					turnLeft(2.5);
					straight(4);
				}
				else{
					straight(0.75);
					turnLeft(1);
					straight(2);
					turnRight(2.5);
					straight(4);
				}
}
	void AutonomousPeriodic(){
	double time=m_time.Get();
	turnangle=position.GetFusedHeading( );//angle robot loses as it moves
		speed=0.25;//-------------change default speed
	// todo use field positions
if 	(fieldData[0]=='L'){//---------------if Left
	if(right){
	PositionIsRight();}
	if (left)
	PositionisLeft();
	if(center)
	PositionisCenter();
		if(time<=3){
					pull();
					raiseElev(24);//24 inches equals 2 feet
				}
			else{
				if(time<=5){
					gripangle.Set(DoubleSolenoid::kForward);//??
				}
				else
				if (time<=6){
					shoot();
				}
			}


				myRobot.CurvatureDrive(-speed, 0.03*((Newstartingangle)-turnangle), false);//?

	}
else{//---------------if Right
	if(right){
	PositionIsRight();}
	if (left)
	PositionisLeft();
	if(center)
	PositionisCenter();
	if(time<=3){
		pull();
		raiseElev(24);//24 inches equals 2 feet
	}
else{
	if(time<=5){
		gripangle.Set(DoubleSolenoid::kForward);//??
	}
	else
	if (time<=6){
		shoot();
	}
				}
					myRobot.CurvatureDrive(-speed, 0.03*((Newstartingangle)-turnangle), false);//?

}}
	double freezeramp=1.5;
	void TeleopInit(){
		m_time.Stop();
		m_time.Reset();
		m_frontleft.EnableVoltageCompensation(false);
		m_frontright.EnableVoltageCompensation(false);
		m_backleft.EnableVoltageCompensation(false);
		m_backright.EnableVoltageCompensation(false);

	}

	/**
	 * Runs the motors with either arcade steering or tank steering.
	 */
	void TeleopPeriodic() {

if(stick.GetRawButton(9))
	tankdrive=true;
else if(stick.GetRawButton(10))
	tankdrive=false;
//------------------------------------//
	if(tankdrive)
		myRobot.TankDrive(stick.GetRawAxis(1),stick.GetRawAxis(3));
    else
    	myRobot.ArcadeDrive(stick.GetY(),-stick.GetX()); // drive with arcade style (use right stick)
 //------------------------------------//
	grip.Set(Ostick.GetY());//check for reverse movement. forward joystick should move box forward
	if(Ostick.GetRawButton(11)){
		gripangle.Set(DoubleSolenoid::kForward);
	}
	else{
				gripangle.Set(DoubleSolenoid::kReverse);

	};
	elev.Set(Ostick.GetRawAxis(3));//check for reverse movement. forward joystick should move elevator up
	//----------------EndGameEndGameEndGameEndGameEndGameEndGameEndGame-------------------------//
	leftwing.checkReleaseTimer();
	rightwing.checkReleaseTimer();
	if (Ostick.GetRawButton(9)
			&&DriverStation::GetInstance().GetMatchTime()<=30){//SAFETY!! ANYTIHNG THAT IS IN THIS FUNCION >>>>MUST<<<<< BE OPERATED USING BUTTON >>9<< OF THE OPERATOR JOYSTICK
		if (Ostick.GetRawButton(7))leftwing.ReleaseWing();//drop left wing
		if (Ostick.GetRawButton(8))rightwing.ReleaseWing();//drop right wing
		if ( Ostick.GetRawButton(5))leftwing.RaiseRamp();
		if ( Ostick.GetRawButton(6))rightwing.RaiseRamp();
		}
	}
	//-------------------------------------------------------------//
};

START_ROBOT_CLASS(Robot)

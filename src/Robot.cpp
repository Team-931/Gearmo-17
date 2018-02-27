//G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17G17
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Drive/DifferentialDrive.h>
//#define usingtank
const double ramptime=0.5;
const double motorvoltage=11.5;//minimum accepted motor voltage for voltage compensation
class wing{
	frc::Solenoid lift1, lift2/*,lift3,lift4*/;
	frc::Servo drop;
	frc::Timer m_freezetime;
bool isrampreleased;
int wingnumber;
public:
	wing(int module_number):lift1(0, 4+2*(module_number)),
							lift2(0, 5+2*(module_number)),
							//lift3(module_number, 6),
							//lift4(module_number, 7),
							drop(module_number),
							isrampreleased(false),
							wingnumber(module_number){}

	void wingUp(){
		if(wingnumber)
		drop.Set(.56);
		else
	drop.Set(0.5);}
	void ReleaseWing(){
		isrampreleased=true;
		if(wingnumber)
				drop.Set(0.56-.33);
				else
			drop.Set(.83);
		m_freezetime.Start();}
	void RaiseRamp(){

		lift1.Set(true);
		lift2.Set(true);
		}
	void Nothing(){
		lift1.Set(false);
		lift2.Set(false);
	}
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
	frc::DifferentialDrive ImperialShuttle { m_left,m_right };// robot drive system
    frc::DoubleSolenoid gripangle{0,0,1};
    ::wing leftwing{0};
    ::wing rightwing{1};

    frc::DigitalInput LowerLimit{0}, UpperLimit{1};//Part of test

	::PigeonIMU position{&m_backright};//This sensor is attached to the controller -- it must initialize after it.
	frc::Joystick stick { 0 };  // driver stick
	frc::Joystick Ostick{ 1 };  // operator stick
	frc::Timer m_time;

	std::string fieldData; // stores the value from GetGameSpecificMessage
	bool blue{true};
		bool center{true};//--
		bool left{true};//-----these can be used  to tell the robot what side it is on
		bool right{true};//---

	// Elevator height settings:
		bool elevZeroed{false};
		int elevBottom{0}, elevMiddle{0}, elevTop{0}; //TODO: better defaults

		void ZeroElev(){
			if(!elevZeroed){
				elevBottom=elev.GetSelectedSensorPosition(0);
				elevZeroed = true;}}

	void safeSet_speed(double speed){
		if(UpperLimit.Get()==false&&speed<=0){
		speed=0;
		}
		if(LowerLimit.Get()==false&&speed>=0){
				speed=0;
		}
		elev.Set(speed);
		}
	void safeSet_position(double height){
		double curheight=elev.GetSelectedSensorPosition(0);
			if(UpperLimit.Get()==false&&height<=curheight){
			height=curheight;
			}
			if(LowerLimit.Get()==false&&height>=curheight){
					height=curheight;
			}
			elev.Set(ControlMode::Position, height);
			}
public:
	Robot() {
		ImperialShuttle.SetExpiration(0.1);
	}
	void RobotInit(){
		elev.SetNeutralMode(Brake);
		grip1.SetInverted(true);
		elev.ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute,0,0);
		elev.ConfigClosedloopRamp(.25, 0);
		elev.SetSensorPhase(false);
		//elev.SelectProfileSlot(0, 0);
		//elev.Config_kP(0, .000244, 0);

	//printf("Entering RobotInit\n");
		leftwing.wingUp();
		rightwing.wingUp();
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

	frc::SmartDashboard::SetDefaultNumber("Elev. bottom value:", elevBottom);
	frc::SmartDashboard::SetDefaultNumber("Elev. middle value:", elevMiddle);
	frc::SmartDashboard::SetDefaultNumber("Elev. top value:", elevTop);
	frc::SmartDashboard::SetDefaultBoolean("Left", false);
	frc::SmartDashboard::SetDefaultBoolean("Right", false);
	frc::SmartDashboard::SetDefaultBoolean("Center", false);

	}

	void DisabledInit(){
		//elevBottom = frc::SmartDashboard::GetNumber("Elev. bottom value:", elevBottom);
		elevMiddle = frc::SmartDashboard::GetNumber("Elev. middle value:", elevMiddle);
		elevTop= frc::SmartDashboard::GetNumber("Elev. top value:", elevTop);
		}

	void AutonomousInit(){
		m_time.Reset();
		m_time.Start();
		leftwing.wingUp();
		rightwing.wingUp();

		ZeroElev();

		startingangle=position.GetFusedHeading();// Which way are we facing
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
		// (m_time.Get()<=timevariable)
			 Newstartingangle=startingangle-turningangle;
	}
	void turnRight(double timevariable){
	//	if (m_time.Get()<=timevariable)
		 Newstartingangle=startingangle-turningangle;

	}
	void straight(double timevariable, double robotspeed){
						m_time.Start();
						if (m_time.Get()<=timevariable){
							 Newstartingangle=startingangle;}
						else robotspeed=0;
						ImperialShuttle.CurvatureDrive(robotspeed, 0.01*(Newstartingangle-turnangle), false);
						if(m_time.Get()>=timevariable)
							m_time.Stop();
							m_time.Reset();}


	void shoot(){
		grip.Set(1);//EDITABLE
	}
	void pull(){
		grip.Set(-.25);
	}

	bool elevHeld{false};
	int elevHold;

	void raiseElev(double height){
//		const double diameter=1.5;
//		static double rotation=elev.GetSelectedSensorPosition(0);
//		static double circumference/*in inches*/=3.141592653585898*(diameter+((rotation/3)*3/16));// doubles only hold 15 bits
//		elev.Set(ControlMode::Position,height/circumference);//EDITABLE
		//elev.Set(ControlMode::Position,elevBottom-4096*60*height);
	}
	void PositionIsRight(){
		if (fieldData[0]=='L'){
			if(m_time.Get()<=3)
				straight(0,0);
			else if (m_time.Get()<=7)
				turnRight(0);
/*			else
				if(m_time.Get()<=1)
			turnLeft(1);
				else
				if(m_time.Get()<=3)
			straight(3);
				else
				if(m_time.Get()<=4)
			turnRight(4);
				else
				if(m_time.Get()<=5)
			straight(5);*/
				else
					speed=0;
		}
		else{
			straight(3,0);
			turnLeft(0.5);
		}
	}
void PositionisLeft(){
	if (fieldData[0]=='R'){
				straight(0.75,0);
				turnLeft(1);
				straight(3,0);
				turnRight(4);
				straight(5,0);
			}
			else{
				straight(3,0);
				turnRight(0.5);
			}
}
void PositionisCenter(){

	if (fieldData[0]=='R'){
					straight(0.75,0);
					turnRight(1);
					straight(2,0);
					turnLeft(2.5);
					straight(4,0);
				}
				else{
								if(m_time.Get()<=1)
					straight(0.75,0);
							//	else
						//					if(m_time.Get()<=1)
					//turnLeft(1);
						//					else
					//									if(m_time.Get()<=2)
				//straight(2);
							//							else
						//											if(m_time.Get()<=2.5)
					//turnRight(2.5);
				//													else
																				if(m_time.Get()<=4)
					//straight(4);
				//																else
																							speed=0;
				}
}
void AutonomousPeriodic(){

}

	/*void AutonomousPeriodic(){

	double time=m_time.Get();
	turnangle=position.GetFusedHeading( );//angle robot loses as it moves
		speed=0.5;//-------------change default speed
	// todo use field positions
if 	(fieldData[0]=='L'){//---------------if Left
	::SmartDashboard::PutNumber("center left time elapsed" ,4*center + 2*left + right);
PositionIsRight();

	if(right){
	PositionIsRight();}
	else if (left)
	PositionisLeft();
	else if(center)
	PositionisCenter();
	else
		speed=0;

		if(time<=3){
				//	pull();
					raiseElev(.02);
					//safeSet_position(20860);
				}
			else{
				if(time<=5){
					gripangle.Set(DoubleSolenoid::kForward);//??
				}
				else
				if (time<=6){
				//	shoot();
				}
			}

double proportion=-0.01*((Newstartingangle)-turnangle);
if(proportion>1)proportion=.2;
if (proportion<-1)proportion=-.2;
				ImperialShuttle.CurvatureDrive(speed, proportion, false);//?

	}
else{//---------------if Right
	if(right){
	PositionIsRight();}
	if (left)
	PositionisLeft();
	if(center)
	PositionisCenter();
	else
		speed=0;
	if(time<=3){
		pull();
		raiseElev(0.02);//24 inches equals 2 feet
		//safeSet_position(24);
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
					ImperialShuttle.CurvatureDrive(speed, -0.1*((Newstartingangle)-turnangle), false);//?

}}*/
	double freezeramp=1.5;
	void TeleopInit(){
		elevHeld = false;
		m_time.Stop();
		m_time.Reset();
		m_frontleft.EnableVoltageCompensation(false);
		m_frontright.EnableVoltageCompensation(false);
		m_backleft.EnableVoltageCompensation(false);
		m_backright.EnableVoltageCompensation(false);
		leftwing.wingUp();
		rightwing.wingUp();

		ZeroElev();
	}

	/**
	 * Runs the motors with either arcade steering or tank steering.
	 */
	void TeleopPeriodic() {


//Part of test
		if(Ostick.GetRawButton(4))
		{frc::SmartDashboard::PutBoolean("Lower switch:", LowerLimit.Get());
		 frc::SmartDashboard::PutBoolean("Upper switch:", UpperLimit.Get());
		 double foo = elev.GetSelectedSensorPosition(0);
		 frc::SmartDashboard::PutNumber("Adjusted encoder:", (elevBottom - foo)/4096/60);
		 frc::SmartDashboard::PutNumber("Encoder:", foo);
		 frc::SmartDashboard::PutNumber("Elev. speed:", elev.GetSelectedSensorVelocity(0));}
//end test
double shiftspeed;
if(stick.GetRawButton(8))
	shiftspeed=1;
else
shiftspeed=0.5;//TODO
if(stick.GetRawButton(9))
	tankdrive=true;
else if(stick.GetRawButton(10))
	tankdrive=false;
//------------------------------------//
	if(tankdrive)
		ImperialShuttle.TankDrive(shiftspeed*-stick.GetRawAxis(1),shiftspeed*-stick.GetRawAxis(3));
    else
    	ImperialShuttle.ArcadeDrive(shiftspeed*-stick.GetY(),shiftspeed*stick.GetX()); // drive with arcade style (use right stick)
 //------------------------------------//
	grip.Set(-Ostick.GetY());
	if(Ostick.GetPOV(0)==0){
		gripangle.Set(DoubleSolenoid::kReverse);
	}
	else{
				gripangle.Set(DoubleSolenoid::kForward);

	}

	//float elevinput=Ostick.GetRawAxis(3);
	//if(abs(elevinput)>(1.0/32)){
	//	safeSet_speed(elevinput);
	//	elevHeld = false;
	//}
	//else if (!elevHeld){
	//	 elevHold = elev.GetSelectedSensorPosition(0);
	//	 elev.Set(ControlMode::Position, elevHold);
	//	 elevHeld = true;
	//}

	//elevator code;
	if(Ostick.GetRawButton(3)) elevHeld = false;
	if(Ostick.GetRawButton(1))
		/*if (elevHeld) elevHeld = false;
		else*/ {elevHeld = true;
			  elevHold = elev.GetSelectedSensorPosition(0);
			  elev.Set(ControlMode::Position, elevHold);}
			///*elev.Set(ControlMode::Position,elevBottom - 20);
	//else if(Ostick.GetRawButton(2)) elev.Set(ControlMode::Position,elevMiddle);
	//else if(Ostick.GetRawButton(3)) elev.Set(ControlMode::Position,elevTop);*/
	else if(!elevHeld)safeSet_speed(Ostick.GetRawAxis(3)); //check for reverse movement. forward joystick should move elevator up
	//----------------EndGameEndGameEndGameEndGameEndGameEndGameEndGame-------------------------//
	leftwing.checkReleaseTimer();
	rightwing.checkReleaseTimer();
	if (Ostick.GetRawButton(9)
			&&DriverStation::GetInstance().GetMatchTime()<=30){//SAFETY!! ANYTIHNG THAT IS IN THIS FUNCION >>>>MUST<<<<< BE OPERATED USING BUTTON >>9<< OF THE OPERATOR JOYSTICK
		if (Ostick.GetRawButton(7))leftwing.ReleaseWing();//drop left wing
		if (Ostick.GetRawButton(8))rightwing.ReleaseWing();//drop right wing
		if ( Ostick.GetRawButton(5))
				leftwing.RaiseRamp();
		else
				leftwing.Nothing();
		if ( Ostick.GetRawButton(6))
				rightwing.RaiseRamp();
		else
				rightwing.Nothing();
		}

	}
	//-------------------------------------------------------------//
};

START_ROBOT_CLASS(Robot)

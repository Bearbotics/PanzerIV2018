#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <ADXRS450_Gyro.h>

#include "Robot.h"

class Robot: public frc::TimedRobot {
public:
	void RobotInit() {
		positionChooser.AddDefault(autoNameDefault, autoNameDefault);
		positionChooser.AddObject(autoNameLeft, autoNameLeft);
		positionChooser.AddObject(autoNameRight, autoNameRight);

		frc::SmartDashboard::PutData("Auto Modes", &positionChooser);
		FrontRightTalon = new WPI_TalonSRX(kFrontRightMotorChannel);
		FrontLeftTalon = new WPI_TalonSRX(kFrontLeftMotorChannel);
		RearLeftTalon = new WPI_TalonSRX(kRearLeftMotorChannel);
		RearRightTalon = new WPI_TalonSRX(kRearRightMotorChannel);

		robotDrive = new MecanumDrive(*FrontLeftTalon, *RearLeftTalon,
				*FrontRightTalon, *RearRightTalon);
		ScissorTalon1 = new TalonSRX(kScissorMotor1Channel);
		ScissorTalon2 = new TalonSRX(kScissorMotor2Channel);

		ScissorTalon2->Set(ControlMode::Follower, kScissorMotor1Channel);
		ScissorTalon1->SetInverted(true);


		GrabberTalon = new TalonSRX(kGrabberMotorChannel);
		ClimberTalon = new TalonSRX(kClimberMotorChannel);
		gyro = new ADXRS450_Gyro();
		CameraServer::GetInstance()->StartAutomaticCapture();

	}





	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit() override {
		autoSelected = positionChooser.GetSelected();
		std::string autoSelected = SmartDashboard::GetString("Auto Selector",
				autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		currentAutoPhase = strafe;


		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		 if (gameData.length > 0) {
			if (autoSelected == autoNameRight) {
			// Right Auto goes here
				if(autoObjective == autoObjectiveLine){
					if(gameData[0]=="L"){
						strafePhaseMultiplier=-3;
						drivePhaseMultiplier=1;}
					else {
						strafePhaseMultiplier=-1;
						drivePhaseMultiplier=1;
					}
				}
				else{
					if(gameData[1]=="L"){
						strafePhaseMultiplier=-4;
						drivePhaseMultiplier=2;
						rotatePhaseMultipler=1;
						scissorPhaseMultiplier=1;}
					else {
						drivePhaseMultiplier=2;
						rotatePhaseMultipler=-1;
						scissorPhaseMultiplier=1;
					}
				}
			}
			else if (autoSelected == autoNameLeft) {
			// Left Auto goes here
				if(autoObjective==0){
					if(gameData[0]=="L"){
						strafePhaseMultiplier=1;
						drivePhaseMultiplier=1;
					}
					else {
						strafePhaseMultiplier=3;
						drivePhaseMultiplier=1;
					}
				}
				else{
					if(gameData[1]=="L"){
						drivePhaseMultiplier=2;
						rotatePhaseMultipler=1;
						scissorPhaseMultiplier=1;
					}
					else {
						strafePhaseMultiplier=4;
						drivePhaseMultiplier=2;
						rotatePhaseMultipler=-1;
						scissorPhaseMultiplier=1;
					}
				}
			}
			else if (autoSelected == autoNameDefault){
			// Center
				if(autoObjective==0){
					if(gameData[0]=="L"){}
					else {}
				}
				else{
					if(gameData[1] == "L"){}
					else {}
				}
		}
		else {
			//Default auto here
			drivePhaseMultiplier=1;
			cubeRotatePhaseMultiplier=0;
			cubeReleasePhaseMultiplier=0;
		}
	}


	}

	void ResetAutoVariables() {
		// Resets all the auto variables for switching phases.
		autoXInput = 0.0;
		autoYInput = 0.0;
		autoZInput = 0.0;
		autoScissorInput = 0.0;

	}
	void CalculateObjective() {
		// The program needs to calculate the objective and set the proper variables for the auto
		//first, reset the variables to zero. this prevents stray motions
		ResetAutoVariables();

		switch(currentAutoPhase) {
		case strafe: {
			//FIXME: add directions
			autoYInput = 1.0;
			targetAutoTick = 100 * strafePhaseMultiplier;
			break;
		}
		case drive: {
			autoXInput = 1.0;
			targetAutoTick = 100 * drivePhaseMultiplier;
			break;
		}
		case rotate: {
			autoZInput = 1.0; //FIXME: DIRECTION
			targetAutoTick = 100 * abs(rotatePhaseMultipler);
			break;
		}
		case scissor: {
			//TODO: implement scissor phase;
			break;
		}
		case cubeRotate: {
			//TODO: implement
			break;
		}
		case cubeRelease: {
			//TODO: implement
			break;
		}
		default:
			std::cout << "This should never happen" << std::endl;
			printf("this should never happen");
			break;
		}

	}

	void AutonomousPeriodic(){
		frc::SmartDashboard::PutString("Auto Phase", AutoPhaseToString(currentAutoPhase));
		if (currentAutoTick >= targetAutoTick) { // if we run out of time
			CalculateObjective(); // recalculate
			currentAutoTick = 0;
		}
		//main drive
		robotDrive->DriveCartesian(autoXInput, autoYInput, autoZInput);
		// special inputs
		currentAutoTick++;
	}


	void TeleopInit() {

	}

	void TeleopPeriodic() {
		// display


		//Motor driving
		if (!leftStick.GetRawButton(1)) {
			robotDrive->DriveCartesian(rightStick.GetX(),
					-1 * rightStick.GetY(), leftStick.GetX());
		} else {
			robotDrive->DriveCartesian(rightStick.GetX(),
					-1 * rightStick.GetY(), leftStick.GetX(), gyro->GetAngle());
		}

		// manipulators
		float scissorSpeed = 0.0;
		float rawScissorSpeed = 0.0;
		//

		if (leftStick.GetRawButton(3)) {
			scissorSpeed = 1 * (leftStick.GetZ() - 1) / 2; //Chris attempts to code part 1
		}
		 else if (leftStick.GetRawButton(2)) {
			scissorSpeed = -1 * (leftStick.GetZ() - 1) / 2; // Part 2
		}
		ScissorTalon1->Set(ControlMode::PercentOutput, scissorSpeed);
/*
		if (rightStick.GetRawButton(3)) {
			GrabberTalon->Set(ControlMode::PercentOutput, 0.5);
		} else if (rightStick.GetRawButton(4)) {
			GrabberTalon->Set(ControlMode::PercentOutput, -0.5);
		} else {
			GrabberTalon->Set(ControlMode::PercentOutput, 0);
		} */

		GrabberTalon->Set(ControlMode::PercentOutput, leftStick.GetY());


		if (rightStick.GetRawButton(2)) {
			ClimberTalon->Set(ControlMode::PercentOutput, rightStick.GetThrottle());
		} else {
			ClimberTalon->Set(ControlMode::PercentOutput, 0.0);
		}

		testingSolenoid.Set(rightStick.GetRawButton(1));
	}

	void TestPeriodic() {
	}


	enum autoPhase { strafe, drive, rotate, scissor, cubeRotate, cubeRelease };

	// tostring for that enum
	std::string AutoPhaseToString(autoPhase autophase) {
		switch(autophase) {
		case strafe: return "strafe";
		case drive: return "drive";
		case rotate: return "rotate";
		case scissor: return "scissor";
		case cubeRotate: return "cubeRotate";
		case cubeRelease: return "cubeRelease";
		default: return "None";
		}
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> positionChooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameLeft = "Left Side";
	const std::string autoNameRight = "Right Side";
	std::string autoSelected;

	WPI_TalonSRX *FrontLeftTalon;
	WPI_TalonSRX *FrontRightTalon;
	WPI_TalonSRX *RearLeftTalon;
	WPI_TalonSRX *RearRightTalon;
	Joystick rightStick { 1 };
	Joystick leftStick { 0 };

	MecanumDrive *robotDrive;

	TalonSRX *ScissorTalon1;

	TalonSRX *GrabberTalon;

	TalonSRX *ClimberTalon;


	std::string gameData;

	ADXRS450_Gyro *gyro;

	frc::Solenoid testingSolenoid {0};
	Compressor *c = new Compressor(0);




	// the numbers are essentially multipliers of the time it needs to drive / the direction`
	//int autoObjective = 0;
	std::string autoObjective = "Line";
	frc::SendableChooser<std::string> objectiveChooser;

	const std::string autoObjectiveLine = "Line";
	const std::string autoObjectiveSwitch = "Switch";
	const std::string autoObjectiveScale = "Scale";

	float strafePhaseMultiplier = 0; //strafe phase
	float drivePhaseMultiplier = 0; //forward phase
	float rotatePhaseMultipler = 0; //turn phase
	float scissorPhaseMultiplier = 0; //scissor phase
	float cubeRotatePhaseMultiplier = 1; //cube rotate phase
	float cubeReleasePhaseMultiplier = 1; //cube release phase

	// Cached results from auto periodic code. This makes it *really* fast.
	float autoXInput = 0.0;
	float autoYInput = 0.0;
	float autoZInput = 0.0;
	float autoScissorInput = 0.0;
	// Auto Timing stuff.
	int currentAutoTick = 0;
	int targetAutoTick = 0;

	autoPhase currentAutoPhase = strafe;



};



START_ROBOT_CLASS(Robot);

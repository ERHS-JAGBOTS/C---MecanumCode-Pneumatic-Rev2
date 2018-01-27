#include <iostream>
#include <string>

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <IterativeRobot.h>
/*#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>*/

class Robot : public frc::IterativeRobot {

	WPI_TalonSRX 	*leftFront 	= new WPI_TalonSRX(0),
					*rightFront	= new WPI_TalonSRX(1),
					*leftRear	= new WPI_TalonSRX(2),
					*rightRear	= new WPI_TalonSRX(3),
					*raise		= new WPI_TalonSRX(4),
					*cubeLeft	= new WPI_TalonSRX(5),
					*cubeRight	= new WPI_TalonSRX(6);

	Joystick		*leftStick	= new Joystick(0),
					*rightStick	= new Joystick(1);

	JoystickButton	*leftButton			= new JoystickButton(leftStick, 2),
					*rightTrigger		= new JoystickButton(rightStick, 1);

	DigitalInput	*limitSwitchLow		= new DigitalInput(7),
					*limitSwitchHigh	= new DigitalInput(8);

	AnalogGyro		*gyro		= new AnalogGyro(0);

	Compressor		*comp		= new Compressor(0);

	Solenoid		*lift		= new Solenoid(0);

	MecanumDrive	*myRobot;

	bool					rightTrigActive,
							leftButtonActive,
							isUpQueued 		= false,
							isDownQueued 	= false,
							goingUp 		= false,
							goingDown 		= false,
							upperLimitReached,
							lowerLimitReached;

public:
	void RobotInit()
	{
		leftFront->	Set(ControlMode::PercentOutput, 0);
		rightFront->Set(ControlMode::PercentOutput, 0);
		leftRear->	Set(ControlMode::PercentOutput, 0);
		rightRear->	Set(ControlMode::PercentOutput, 0);

		rightFront->SetInverted(true);
		rightRear->SetInverted(true);

		myRobot = new MecanumDrive(*leftFront, *rightFront, *leftRear, *rightRear);
		myRobot->SetExpiration(0.5);
		myRobot->SetSafetyEnabled(false);
	}

	void AutonomousPeriodic()
	{
		//monkaS
	}

	double Db(double axisVal)
	{
		if(axisVal < -0.1)
		{
			return axisVal;
		}
		if(axisVal > 0.1)
		{
			return axisVal;
		}
		return 0;
	}

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		float angle = gyro->GetAngle();

		myRobot->DriveCartesian(Db(rightStick->GetX()),
								Db(rightStick->GetY()),
								Db(leftStick->GetZ()),
								angle);

		if(limitSwitchLow->Get())
		{
			lowerLimitReached = true;
		}
		else
		{
			lowerLimitReached = false;
		}

		if(limitSwitchHigh->Get())
		{
			upperLimitReached = true;
		}
		else
		{
			upperLimitReached = false;
		}

		if(leftButton && lowerLimitReached && !isUpQueued
				&& !isDownQueued && !goingUp && !goingDown)
		{
			isUpQueued = true;
		}
		else if(upperLimitReached)
		{
			isUpQueued = false;
		}

		if(isUpQueued && !upperLimitReached)
		{
			raise->Set(0.5);
			goingUp = true;
			goingDown = false;
		}
		else if(upperLimitReached)
		{
			raise->Set(0);
			goingUp = false;
		}

		if(leftButton && upperLimitReached && !isUpQueued
				&& !isDownQueued && !goingUp && !goingDown)
		{
			isDownQueued = true;
		}
		else if(lowerLimitReached)
		{
			isDownQueued = false;
		}

		if(isDownQueued && !lowerLimitReached)
		{
			raise->Set(-0.5);
			goingUp = false;
			goingDown = true;
		}
		else if(lowerLimitReached)
		{
			raise->Set(0);
			goingDown = false;
		}

		if(rightTrigger)
		{
			cubeLeft->Set(0.3);
			cubeRight->Set(-0.3);
		}
		else if(!rightTrigger)
		{
			cubeLeft->Set(0);
			cubeRight->Set(0);
		}
	}

private:

	//EZ CLAP

};

START_ROBOT_CLASS(Robot)

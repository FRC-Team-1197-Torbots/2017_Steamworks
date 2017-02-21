package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorDrive
{	
	private boolean isHighGear = true;
	private Solenoid m_solenoidshift;
	private Joystick cypress;

	private double rightMotorSpeed;
	private double leftMotorSpeed;
	private TorJoystickProfiles joystickProfile;
	private double targetSpeed;
	private double targetOmega;
	private double trackWidth = 0.5715; //meters, in inches 22.5
	private double halfTrackWidth = trackWidth / 2.0;
	private double centerRadius = 0.0;
	private double maxThrottle;
	
	private boolean buttonYlast;
	private boolean buttonBlast;
	private boolean buttonXlast;
	private boolean buttonAlast;
	
<<<<<<< HEAD
	private TorTrajectory forwardTrajectory;
	private TorTrajectory backwardTrajectory;
	private TorTrajectory rightTrajectory;
	private TorTrajectory leftTrajectory;
	
=======
	private static TorTrajectory forTraj;
	private static TorTrajectory backTraj;
	private static TorTrajectory rightTraj;
	private static TorTrajectory leftTraj;

>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
	private BoilerPos1 boilerPos1;
//	private GearBackTraj traj;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			TorMotionProfile.INSTANCE.run();
		}
	}
	Notifier mpNotifier = new Notifier(new PeriodicRunnable());

	public TorDrive(Joystick stick, Solenoid shift, Joystick cypress)
	{
		TorCAN.INSTANCE.resetEncoder();
		TorCAN.INSTANCE.resetHeading();
//		TorMotionProfile.INSTANCE.resetWaypoints();
		
		forTraj = new LinearTrajectory(0.5);
		backTraj = new LinearTrajectory(-0.5);
		rightTraj = new PivotTrajectory(90);
		leftTraj = new PivotTrajectory(-90);
		
		forwardTrajectory = new LinearTrajectory(1.0);
		backwardTrajectory = new LinearTrajectory(-1.0);
		rightTrajectory = new PivotTrajectory(90);
		leftTrajectory = new PivotTrajectory(-90);
		
		this.cypress = cypress;
<<<<<<< HEAD
//		traj = new GearBackTraj();
=======
		forTraj = new forwardTrajectory();
		traj = new GearBackTraj();
>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
		boilerPos1 = new BoilerPos1();
		joystickProfile = new TorJoystickProfiles();
		
		maxThrottle = (0.6) * (joystickProfile.getMinTurnRadius() / (joystickProfile.getMinTurnRadius() + halfTrackWidth));
		
		m_solenoidshift = shift;
		mpNotifier.startPeriodic(0.005);
	}
	

	public void driving(double throttleAxis, double arcadeSteerAxis, double carSteerAxis, boolean shiftButton, boolean rightBumper,
			boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY){
		//Only switch to carDrive in high gear
		if(isHighGear){
			//carDrive(throttleAxis, carSteerAxis);
			buttonDrive(buttonA, buttonB, buttonX, buttonY);

			//When you hold down the shiftButton (left bumper), then shift to low gear.
			if(shiftButton){
				shiftToLowGear();
			}
		}

		//Only switch to ArcadeDrive in low gear
		else{
			ArcadeDrive(throttleAxis, arcadeSteerAxis);

			//When you release the shiftButton (left bumper), then shift to high gear.
			if(!shiftButton){
				shiftToHighGear();
			}

		}
	}
	
	//Shifts the robot to high gear and change the talon's control mode to speed.
	public void shiftToHighGear(){
		if (!isHighGear){
			m_solenoidshift.set(false);
			TorCAN.INSTANCE.chooseVelocityControl();
			isHighGear = true;
			TorMotionProfile.INSTANCE.setActive();
		}
	}
	
	//Shifts the robot to low gear and change the talon's control mode to percentVbus.
	public void shiftToLowGear(){
		if (isHighGear){
			m_solenoidshift.set(true);
			TorCAN.INSTANCE.choosePercentVbus();
			isHighGear = false;
			TorMotionProfile.INSTANCE.setInactive();
		}
	}

	public void ArcadeDrive(double throttleAxis, double arcadeSteerAxis){
		throttleAxis = -throttleAxis;
		if (Math.abs(arcadeSteerAxis) <= 0.1) {
			arcadeSteerAxis = 0.0D;
		}
		if (Math.abs(throttleAxis) <= 0.2D) {
			throttleAxis = 0.0D;
		}

		if (arcadeSteerAxis >= 0.0D) {
			arcadeSteerAxis *= arcadeSteerAxis;
		} else {
			arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
		}
		if (throttleAxis >= 0.0D) {
			throttleAxis *= throttleAxis;
		} else {
			throttleAxis = -(throttleAxis * throttleAxis);
		}
		
		double rightMotorSpeed;
		double leftMotorSpeed;

		if (throttleAxis > 0.0D)
		{
			if (arcadeSteerAxis > 0.0D)
			{
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = Math.max(throttleAxis, arcadeSteerAxis);
			}
			else
			{
				leftMotorSpeed = Math.max(throttleAxis, -arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
		}
		else
		{
			if (arcadeSteerAxis > 0.0D)
			{
				leftMotorSpeed = -Math.max(-throttleAxis, arcadeSteerAxis);
				rightMotorSpeed = throttleAxis + arcadeSteerAxis;
			}
			else
			{
				leftMotorSpeed = throttleAxis - arcadeSteerAxis;
				rightMotorSpeed = -Math.max(-throttleAxis, -arcadeSteerAxis);
			}
		}
		TorCAN.INSTANCE.SetDrive(rightMotorSpeed, leftMotorSpeed);
	}
	
	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY){
		if(buttonB && !buttonBlast){
<<<<<<< HEAD
			TorMotionProfile.INSTANCE.executeTrajectory(rightTrajectory);
=======
			TorMotionProfile.INSTANCE.executeTrajectory(rightTraj);
>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
		}
		else if(buttonX && !buttonXlast){
<<<<<<< HEAD
			TorMotionProfile.INSTANCE.executeTrajectory(leftTrajectory);
=======
			TorMotionProfile.INSTANCE.executeTrajectory(leftTraj);
>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
		}
		else if(buttonY && !buttonYlast){
<<<<<<< HEAD
			TorMotionProfile.INSTANCE.executeTrajectory(forwardTrajectory);
//			forwardTrajectory.execute();
=======
			TorMotionProfile.INSTANCE.executeTrajectory(forTraj);
>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
		}
		else if(buttonA && !buttonAlast){
<<<<<<< HEAD
			TorMotionProfile.INSTANCE.executeTrajectory(backwardTrajectory);
=======
			TorMotionProfile.INSTANCE.executeTrajectory(backTraj);
>>>>>>> branch 'master' of https://github.com/FRC-Team-1197-Torbots/2017_Steamworks.git
		}
		else{
			
		}
		buttonBlast = buttonB;
		buttonYlast = buttonY;
		buttonXlast = buttonX;
		buttonAlast = buttonA;
	}
	
	public void carDrive(double throttleAxis, double carSteeringAxis){
		//Flipping the sign so it drives forward when you move the analog stick up and vice versa
		throttleAxis = -throttleAxis;
		carSteeringAxis = -carSteeringAxis;

		targetSpeed = joystickProfile.findSpeedSimple(throttleAxis) * TorCAN.INSTANCE.absoluteMaxSpeed();
		targetSpeed *= maxThrottle;

		/* The centerRadius is the value we gain from findRadiusExponential method in the joystickProfile class.
		   The TorMath.sign(throttleAxis) makes steering work the same when the robot drives backwards. */
		centerRadius = TorMath.sign(throttleAxis) * joystickProfile.findRadiusExponential(carSteeringAxis);

		// If the centerRadius is greater than the maxTurnRadius or if it is 0, then drive straight:
		if (Math.abs(centerRadius) > joystickProfile.getMaxTurnRadius() || Math.abs(centerRadius) == 0.0)
		{
			targetOmega = 0.0;
		}
		// Else, steer:
		else {
			targetOmega = targetSpeed / centerRadius;
		}

		// Setting the joystick trajectory targets so that it actually drives:
		TorMotionProfile.INSTANCE.joystickTraj.setTargets(targetSpeed, targetOmega);
		SmartDashboard.putNumber("targetSpeed", targetSpeed);
	}

}

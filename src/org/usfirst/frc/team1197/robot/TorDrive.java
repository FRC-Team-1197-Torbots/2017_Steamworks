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

	private static TorTrajectory forwardTrajectory;
	private static TorTrajectory rightTrajectory;
	private static TorTrajectory backwardTrajectory;
	private static TorTrajectory leftTrajectory;
	
	private testSpline testspline;
	
	private boolean buttonYlast;
	private boolean buttonBlast;
	private boolean buttonXlast;
	private boolean buttonAlast;
	
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
		this.cypress = cypress;
		
		joystickProfile = new TorJoystickProfiles();
		forwardTrajectory = new LinearTrajectory(1.0);
		backwardTrajectory = new LinearTrajectory(-1.0);
		rightTrajectory = new PivotTrajectory(90);
		leftTrajectory = new PivotTrajectory(-90);
		
		testspline = new testSpline();
		
		maxThrottle = (0.6) * (joystickProfile.getMinTurnRadius() / (joystickProfile.getMinTurnRadius() + halfTrackWidth));
		
		m_solenoidshift = shift;
		mpNotifier.startPeriodic(0.005);
	}
	
	
	public void driving(double throttleAxis, double arcadeSteerAxis, double carSteerAxis, boolean shiftButton, boolean rightBumper,
			boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY){
		//Only switch to carDrive in high gear
		if(isHighGear){
			if(cypress.getRawButton(1)){
				ArcadeDrive(throttleAxis, arcadeSteerAxis);
				
				if(shiftButton){
					shiftToLowGear();
				}
			}
			else{
//				ackermanDrive(throttleAxis, carSteerAxis);
				buttonDrive(buttonA, buttonB, buttonX, buttonY);
				
				//When you hold down the shiftButton (left bumper), then shift to low gear.
				if(shiftButton){
					shiftToLowGear();
				}
			}
		}
		
		//Only switch to ArcadeDrive in low gear
		else{
			if(cypress.getRawButton(1)){
				ArcadeDrive(throttleAxis, arcadeSteerAxis);
				
				if(!shiftButton){
					shiftToHighGearNoMotion();
				}
			}
			else{
				ArcadeDrive(throttleAxis, arcadeSteerAxis);
				
				//When you release the shiftButton (left bumper), then shift to high gear.
				if(!shiftButton ){
					shiftToHighGearMotion();
				}
			}		
		}
	}
	
	//Shifts the robot to high gear and change the talon's control mode to speed.
	public void shiftToHighGearMotion(){
		if (!isHighGear){
			m_solenoidshift.set(false);
			TorCAN.INSTANCE.chooseVelocityControl();
			isHighGear = true;
//			TorMotionProfile.INSTANCE.joystickTraj.execute();
			TorMotionProfile.INSTANCE.setActive();
		}
	}
	
	public void shiftToHighGearNoMotion(){
		if(!isHighGear){
			m_solenoidshift.set(false);
			TorCAN.INSTANCE.chooseVelocityControl();
			isHighGear = true;
			TorMotionProfile.INSTANCE.setInactive();
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

	
	public void carDrive(double throttleAxis, double carSteeringAxis){
		
		//Flipping the sign so it drives forward when you move the analog stick up and vice versa
		carSteeringAxis = -carSteeringAxis;
		
		targetSpeed = joystickProfile.findSpeed(throttleAxis) * TorCAN.INSTANCE.getSensorSpeed();
		
		targetSpeed *= maxThrottle;
		
		/* The centerRadius is the value we gain from findRadiusExponential method in the joystickProfile class.
		   The Math.abs(throttleAxis) / throttleAxis is to make sure the sign of the centerRadius is right. */
		centerRadius = (Math.abs(throttleAxis) / throttleAxis) * joystickProfile.findRadiusExponential(carSteeringAxis);
		
		//If the centerRadius is greater than the maxTurnRadius or the centerRadius is 0, then drive forward and vice versa.
		if (Math.abs(centerRadius) > joystickProfile.getMaxTurnRadius() || Math.abs(centerRadius) == 0.0)
		{
			leftMotorSpeed = targetSpeed;
			rightMotorSpeed = targetSpeed;
		}
		
		//Else, steer
		else {
			rightMotorSpeed = targetSpeed * ((centerRadius - halfTrackWidth) / centerRadius);
			leftMotorSpeed = targetSpeed * ((centerRadius + halfTrackWidth) / centerRadius);
		}

		//Setting the rightMotorSpeed and the leftMotorSpeed so that it actually drives.
		TorCAN.INSTANCE.SetDrive(rightMotorSpeed, -leftMotorSpeed);
	}
	
	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY){
		if(buttonB && !buttonBlast){
//			rightTrajectory.execute();
		}
		else if(buttonX && !buttonXlast){
//			leftTrajectory.execute();
		}
		else if(buttonY && !buttonYlast){
//			forwardTrajectory.execute();
			testspline.execute();
		}
		else if(buttonA && !buttonAlast){
//			backwardTrajectory.execute();
		}
		else{
			
		}
		buttonBlast = buttonB;
		buttonYlast = buttonY;
		buttonXlast = buttonX;
		buttonAlast = buttonA;
	}
	
	public void ackermanDrive(double throttleAxis, double carSteeringAxis){
		//Flipping the sign so it drives forward when you move the analog stick up and vice versa
		throttleAxis = -throttleAxis;
		carSteeringAxis = -carSteeringAxis;

		targetSpeed = joystickProfile.findSpeedSimple(throttleAxis) * TorCAN.INSTANCE.absoluteMaxSpeed();
//		targetSpeed = joystickProfile.findSpeedSimple(throttleAxis) * 4.405;
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

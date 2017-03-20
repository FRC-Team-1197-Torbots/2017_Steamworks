package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TorDrive
{	
	public final DriveController controller;
	private Joystick cypress;
	private TorJoystickProfiles joystickProfile;

	private double targetSpeed;
	private double targetOmega;
	private double centerRadius;
	private double maxThrottle;
	
	private boolean buttonYlast;
	private boolean buttonBlast;
	private boolean buttonXlast;
	private boolean buttonAlast;
	
	private double dangerFactor = 0.75;
	
	private double v;
	private double w;
	
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			controller.run();
		}
	}
	Notifier mpNotifier = new Notifier(new PeriodicRunnable());

	public TorDrive(Joystick stick, Joystick cypress)
	{
		controller = new DriveController(false);
		this.cypress = cypress;
		joystickProfile = new TorJoystickProfiles();
		//TODO: Use static final in TorJoystickProfile, then remove maxThrottle initialization from the constructor.
		maxThrottle = (dangerFactor) * (joystickProfile.getMinTurnRadius() 
				/ (joystickProfile.getMinTurnRadius() + DriveHardware.halfTrackWidth));
		mpNotifier.startPeriodic(0.005);
	}
	
	public TorDrive()
	{
		controller = new DriveController(true);
		joystickProfile = new TorJoystickProfiles();
		//TODO: Use static final in TorJoystickProfile, then remove maxThrottle initialization from the constructor.
		maxThrottle = (dangerFactor) * (joystickProfile.getMinTurnRadius() 
				/ (joystickProfile.getMinTurnRadius() + DriveHardware.halfTrackWidth));
		mpNotifier.startPeriodic(0.005);
	}


	public void driving(double throttleAxis, double arcadeSteerAxis, double carSteerAxis, boolean shiftButton,
			boolean rightBumper, boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY) {
		// Tell the drive controller whether to use car drive in high gear.
		controller.useCarDriveInHighGear(!cypress.getRawButton(1));
		if (controller.isHighGear) {
			if (controller.usingCarDriveForHighGear) {
//				carDrive(throttleAxis, carSteerAxis);
				ImprovedArcadeDrive(throttleAxis, arcadeSteerAxis);
//				buttonDrive(buttonA, buttonB, buttonX, buttonY);
			} else {
				ArcadeDrive(throttleAxis, arcadeSteerAxis);
//				ImprovedArcadeDrive(throttleAxis, arcadeSteerAxis);
			}
			// When you hold down the shiftButton (left bumper), then shift to low gear.
			if (shiftButton) {
				controller.shiftToLowGear();
			}
		} else {
			ArcadeDrive(throttleAxis, arcadeSteerAxis);
			// When you release the shiftButton (left bumper), then shift to high gear.
			if (!shiftButton) {
				controller.shiftToHighGear();
			}
		}
	}
	
	public void enable() {
		controller.useCarDriveInHighGear(!cypress.getRawButton(1));
		controller.enable();
	}

	public void disable() {
		controller.disable();
	}
	
	public void ImprovedArcadeDrive(double throttleAxis, double arcadeSteerAxis){
		
		if (Math.abs(arcadeSteerAxis) <= 0.2) {
			arcadeSteerAxis = 0.0;
		}
		if (Math.abs(throttleAxis) <= 0.2) {
			throttleAxis = 0.0;
		}
		
		if (arcadeSteerAxis >= 0.0D) {
			arcadeSteerAxis *= arcadeSteerAxis * arcadeSteerAxis;
		} else {
			arcadeSteerAxis = -(arcadeSteerAxis * arcadeSteerAxis);
		}
		if (throttleAxis >= 0.0D) {
			throttleAxis *= throttleAxis;
		} else {
			throttleAxis = -(throttleAxis * throttleAxis);
		}
		SmartDashboard.putNumber("arcadeSteer", dangerFactor * DriveHardware.absoluteMaxOmega * DriveHardware.halfTrackWidth);
		throttleAxis *= dangerFactor * DriveHardware.absoluteMaxSpeed;
		arcadeSteerAxis *= dangerFactor * DriveHardware.absoluteMaxOmega * DriveHardware.halfTrackWidth;
		
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
		
		v = (rightMotorSpeed + leftMotorSpeed) * 0.5;
		w = (rightMotorSpeed - leftMotorSpeed) / DriveHardware.trackWidth;
	
		SmartDashboard.putNumber("v", v);
		SmartDashboard.putNumber("w", w);
		controller.setTargets(v, w);
	}

	public void ArcadeDrive(double throttleAxis, double arcadeSteerAxis){
//		throttleAxis = -throttleAxis; // TODO: see below.
		arcadeSteerAxis = -arcadeSteerAxis;
		
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
		
//		controller.setTargets(rightMotorSpeed, leftMotorSpeed); // TODO: This is what it was till now. Fix sign issue pls?
		controller.setTargets(leftMotorSpeed, rightMotorSpeed); // (Let's switch to this and use TorJoystickProfiles if we can)
	}

	public void buttonDrive(boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY){
		if(buttonB && !buttonBlast){
			
		}
		else if(buttonX && !buttonXlast){
			
		}
		else if(buttonY && !buttonYlast){
			
		}
		else if(buttonA && !buttonAlast){
			
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
		//TODO: Shouldn't the signs be settled before we get to this point??? (fix in TorJoystickProfiles, not here)
//		throttleAxis = -throttleAxis;
//		carSteeringAxis = -carSteeringAxis;
		targetSpeed = joystickProfile.findSpeedSimple(throttleAxis) * DriveHardware.absoluteMaxSpeed;
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
		SmartDashboard.putNumber("targetSpeed", targetSpeed);
		SmartDashboard.putNumber("targetOmega", targetOmega);
		controller.setTargets(targetSpeed, targetOmega); // TODO: replace with controller.setTargets()
	}

	public void executeTrajectory(TorTrajectory traj) {
		if (controller.motionProfilingActive()) {
			controller.loadTrajectory(traj);
		} else {
			System.err.println("ERROR: Could not execute trajectory with motion profiling inactive.");
		}
	}

}

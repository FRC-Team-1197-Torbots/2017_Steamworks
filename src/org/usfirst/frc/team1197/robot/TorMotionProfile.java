package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.TorPID.sensorLimitMode;
import org.usfirst.frc.team1197.robot.TorPID.sensorNoiseMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum TorMotionProfile 
{
	INSTANCE;
	
	private boolean isActive = false;
	private TorTrajectory activeTrajectory = null;
	private TorTrajectory nextTrajectory = null;
	private TorTrajectory defaultTrajectory = null;
	private final double timeInterval = 0.005;
	
	private double targetVelocity;
	private double targetAcceleration;
	private double targetPosition;
	
	private double targetOmega;
	private double targetAlpha;
	private double targetHeading;

	private final double kPv = 0.35; //0.6
	private final double kA = 0.03; //0.05
	private final double kP = 7.0;  //7.0
	private final double kI = 3.0;  //3.0
	private final double kD = 0.0075;  //0.0075

	private final double kpv = 0.01; //0.25
	private final double ka = 0.01; //0.0
	private final double kp = 19.75; //20.75
	private final double ki = 8.0; //5.0
	private final double kd = 0.03; //0.025
	
	private final double minLineOutput = 0.0; //0.0
	private final double minTurnOutput = 0.3; //0.8

	private double dt = 0.005;
	
	private long currentTime;
	private long lastTime;
	private long lookupTime;
	private long startTime;
	
	public final JoystickTrajectory joystickTraj;
	private final TorPID positionPID;
	private final TorPID headingPID;
	
	protected static double positionWaypoint;
	protected static double headingWaypoint;
	
	private final boolean usingWaypoint = true;
	
	private TorMotionProfile(){
		joystickTraj = new JoystickTrajectory();
		positionPID = new TorPID(dt);
		headingPID = new TorPID(dt);
		
		defaultTrajectory = joystickTraj;
		
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		
		positionPID.setLimitMode(sensorLimitMode.Default);
		positionPID.setNoiseMode(sensorNoiseMode.Noisy);
		positionPID.setBacklash(0.0);
		positionPID.setPositionTolerance(0.01); 
		positionPID.setVelocityTolerance(0.0125);
		positionPID.setMinimumOutput(minLineOutput);
		positionPID.setkP(kP);
		positionPID.setkI(kI);
		positionPID.setkD(kD);
		positionPID.setkPv(kPv);
		positionPID.setkA(kA);
		
		headingPID.setLimitMode(sensorLimitMode.Coterminal);
		headingPID.setNoiseMode(sensorNoiseMode.Noisy);
		headingPID.setBacklash(0.0);
		headingPID.setPositionTolerance(0.0115); //0.015
		headingPID.setVelocityTolerance(0.0125);
		headingPID.setMinimumOutput(minTurnOutput);
		headingPID.setkP(kp);
		headingPID.setkI(ki);
		headingPID.setkD(kd);
		headingPID.setkPv(kpv);
		headingPID.setkA(ka);
	}
	
	public void loadTrajectory(TorTrajectory traj){
		if(!usingWaypoint){
			TorCAN.INSTANCE.resetEncoder();
			TorCAN.INSTANCE.resetHeading();
			resetPID();
		}
		nextTrajectory = traj;
	}
	
	public void setActive(){
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		resetPID();
		isActive = true;
	}
	public void setInactive(){
		isActive = false;
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
	}
	public boolean isActive(){
		return isActive;
	}
	
	public double getTimeInterval(){
		return timeInterval;
	}
	
	public void run(){
		currentTime = System.currentTimeMillis();
		dt = (currentTime - lastTime) * 0.001;
		lastTime = currentTime; 
		currentTime = (currentTime - (currentTime % ((long)(getTimeInterval() * 1000))));
		lookupTime = currentTime - startTime;
		positionPID.updateDt(dt);
		headingPID.updateDt(dt);
		joystickTraj.updateDt(dt); 

		//Position
		positionPID.updatePosition(TorCAN.INSTANCE.getPosition());
		positionPID.updateVelocity(TorCAN.INSTANCE.getVelocity());

		//Heading
		headingPID.updatePosition(TorCAN.INSTANCE.getHeading());
		headingPID.updateVelocity(TorCAN.INSTANCE.getOmega());
		
		//TODO: Set the joystick trajectory's acceleration equal to the active trajectory's acceleration
		if(isActive){
			if(activeTrajectory == joystickTraj){
				joystickTraj.updateVelocity();
				joystickTraj.updateOmega();
			}
		}
		else{
			joystickTraj.setState(positionPID.position(), positionPID.velocity(), headingPID.position(), headingPID.velocity());
		}
		
		targetPosition = activeTrajectory.lookUpPosition(lookupTime);
		if(activeTrajectory != joystickTraj){
			targetPosition += positionWaypoint;
		}
		targetVelocity = activeTrajectory.lookUpVelocity(lookupTime);
		targetAcceleration = activeTrajectory.lookUpAcceleration(lookupTime);
		
		positionPID.updatePositionTarget(targetPosition);
		positionPID.updateVelocityTarget(targetVelocity);
		positionPID.updateAccelerationTarget(targetAcceleration);
		
		targetHeading = activeTrajectory.lookUpHeading(lookupTime);
		if(activeTrajectory != joystickTraj){
			targetHeading += headingWaypoint;
		}
		targetOmega = activeTrajectory.lookUpOmega(lookupTime);
		targetAlpha = activeTrajectory.lookUpAlpha(lookupTime);
		
		headingPID.updatePositionTarget(targetHeading);
		headingPID.updateVelocityTarget(targetOmega);
		headingPID.updateAccelerationTarget(targetAlpha);
		
		positionPID.update();
		headingPID.update();
		
		if(isActive){
			if(activeTrajectory != joystickTraj){
				joystickTraj.setState(targetPosition, targetVelocity, targetHeading, targetOmega);
			}
			graphLinear();
			graphRotational();
			TorCAN.INSTANCE.setTargets(positionPID.output(), headingPID.output());
		}
		SmartDashboard.putBoolean("lookUpIsLast(currentTime)", activeTrajectory.lookUpIsLast(currentTime));
		SmartDashboard.putBoolean("positionPID.isOnTarget()", positionPID.isOnTarget());
		SmartDashboard.putBoolean("headingPID.isOnTarget()", headingPID.isOnTarget());
		if(activeTrajectory.lookUpIsLast(lookupTime) && positionPID.isOnTarget() && headingPID.isOnTarget()){
			startTime = currentTime;
			if(!(activeTrajectory == defaultTrajectory && nextTrajectory == defaultTrajectory)){
				if(usingWaypoint){
					positionWaypoint = targetPosition;
					headingWaypoint = targetHeading;
				}
				activeTrajectory.setComplete(true);
				activeTrajectory = nextTrajectory;
				nextTrajectory = defaultTrajectory;
			}
		}
	}
	
	public void graphLinear(){
		SmartDashboard.putNumber("targetVelocity", targetVelocity);
		SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
		SmartDashboard.putNumber("targetPosition", targetPosition);
		SmartDashboard.putNumber("currentVelocity", positionPID.velocity());
		SmartDashboard.putNumber("currentAcceleration", positionPID.acceleration());
		SmartDashboard.putNumber("currentPosition", positionPID.position());
		SmartDashboard.putNumber("dDispErrordt", positionPID.dErrodt());
		SmartDashboard.putNumber("positionError", positionPID.error());
	}
	
	public void graphRotational(){
		SmartDashboard.putNumber("targetOmega", targetOmega);
		SmartDashboard.putNumber("targetAlpha", targetAlpha);
		SmartDashboard.putNumber("targetHeading", targetHeading);
		SmartDashboard.putNumber("currentOmega", headingPID.velocity());
		SmartDashboard.putNumber("currentAlpha", headingPID.acceleration());
		SmartDashboard.putNumber("currentHeading", headingPID.position());
		SmartDashboard.putNumber("dHeadErrordt", headingWaypoint);
		SmartDashboard.putNumber("headingError", headingPID.error());
	}
	
	public void executeTrajectory(TorTrajectory traj){
		loadTrajectory(traj);
		traj.setComplete(false);
	}
	
	public boolean isComplete(){
		return (activeTrajectory == defaultTrajectory);
	}
	
	public boolean dispOnTarget(){
		return positionPID.isOnTarget();
	}
	
	public boolean headOnTarget(){
		return headingPID.isOnTarget();
	}
	
	public void resetWaypoints(){
		positionWaypoint = 0;
		headingWaypoint = 0;
	}
	
	public void resetPID(){
		headingPID.reset();
		positionPID.reset();
	}
}

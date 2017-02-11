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

	private final double kPv = 0.1; //0.05
	private final double kA = 0.1; //0.05
	private final double kP = 5.0;  //5.25
	private final double kI = 1.5;  //1.75
	private final double kD = 0.01;  //0.05

	private final double kpv = 0.75; //0.1
	private final double ka = 0.1; //0.05
	private final double kp = 25.0; //18.75
	private final double ki = 3.0; //4.0
	private final double kd = 0.01; //0.35
	
	private final double minLineOutput = 0.0; //0.0
	private final double minTurnOutput = 0.5; //0.2

	private double dt = 0.005;
	
	private long currentTime;
	private long lastTime;
	private long lookupTime;
	private long startTime;
	
	public final JoystickTrajectory joystickTraj;
	private final StationaryTrajectory stationaryTraj;
	private final TorPID positionPID;
	private final TorPID headingPID;
	
	protected static double positionWaypoint;
	protected static double headingWaypoint;
	
	private final boolean usingWaypoint = true;
	
	private TorMotionProfile(){
		joystickTraj = new JoystickTrajectory();
		stationaryTraj = new StationaryTrajectory();
		positionPID = new TorPID(dt);
		headingPID = new TorPID(dt);
		
		defaultTrajectory = joystickTraj;
		
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		
		positionPID.setLimitMode(sensorLimitMode.Default);
		positionPID.setNoiseMode(sensorNoiseMode.Noisy);
		positionPID.setBacklash(0.0);
		positionPID.setPositionTolerance(0.02); //0.015
		positionPID.setVelocityTolerance(0.02);
		positionPID.setMinimumOutput(minLineOutput);
		positionPID.setkP(kP);
		positionPID.setkI(kI);
		positionPID.setkD(kD);
		positionPID.setkPv(kPv);
		positionPID.setkA(kA);
		
		headingPID.setLimitMode(sensorLimitMode.Coterminal);
		headingPID.setNoiseMode(sensorNoiseMode.Noisy);
		headingPID.setBacklash(0.0);
		headingPID.setPositionTolerance(0.01);
		headingPID.setVelocityTolerance(0.01);
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
		resetWaypoints();
//		TorCAN.INSTANCE.resetEncoder();
//		TorCAN.INSTANCE.resetHeading();
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
		if(isActive && activeTrajectory == joystickTraj){
			joystickTraj.updateVelocity();
			joystickTraj.updateOmega();
		}
		else{
			joystickTraj.setState(targetPosition, targetVelocity, targetHeading, targetVelocity);
		}
		
		targetPosition = activeTrajectory.lookUpPosition(currentTime);
		if(activeTrajectory != joystickTraj){
			targetPosition += positionWaypoint;
		}
		targetVelocity = activeTrajectory.lookUpVelocity(currentTime);
		targetAcceleration = activeTrajectory.lookUpAcceleration(currentTime);
		
		positionPID.updatePositionTarget(targetPosition);
		positionPID.updateVelocityTarget(targetVelocity);
		positionPID.updateAccelerationTarget(targetAcceleration);
		
		targetHeading = activeTrajectory.lookUpHeading(currentTime);
		if(activeTrajectory != joystickTraj){
			targetHeading += headingWaypoint;
		}
		targetOmega = activeTrajectory.lookUpOmega(currentTime);
		targetAlpha = activeTrajectory.lookUpAlpha(currentTime);
		
		headingPID.updatePositionTarget(targetHeading);
		headingPID.updateVelocityTarget(targetOmega);
		headingPID.updateAccelerationTarget(targetAlpha);
		
		positionPID.update();
		headingPID.update();
		
		if(isActive){
			if(activeTrajectory == joystickTraj){
				joystickTraj.updateVelocity();
				joystickTraj.updateOmega();
			}
			else{
				joystickTraj.setState(positionPID.position(), positionPID.velocity(), headingPID.position(), headingPID.velocity());
			}
			graphLinear();
			graphRotational();
			TorCAN.INSTANCE.setTargets(positionPID.output(), headingPID.output());
		}
		SmartDashboard.putBoolean("lookUpIsLast(currentTime)", activeTrajectory.lookUpIsLast(currentTime));
		SmartDashboard.putBoolean("positionPID.isOnTarget()", positionPID.isOnTarget());
		SmartDashboard.putBoolean("headingPID.isOnTarget()", headingPID.isOnTarget());
		if(activeTrajectory.lookUpIsLast(currentTime) && positionPID.isOnTarget() && headingPID.isOnTarget()){
			startTime = currentTime;
			if(!(activeTrajectory == defaultTrajectory && nextTrajectory == defaultTrajectory)){
				System.out.println("IS ON TARGET!!!!");
				if(usingWaypoint){
					positionWaypoint += activeTrajectory.goalPos();
					headingWaypoint += activeTrajectory.goalHead();
				}
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
	
	public void executeDefault(){
		if(defaultTrajectory == stationaryTraj){
			stationaryTraj.execute();
		}
		else{
			joystickTraj.execute(0.0,0.0,0.0,0.0);
		}
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

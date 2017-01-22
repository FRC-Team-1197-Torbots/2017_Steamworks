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
	private double targetDisplacement;
	
	private double targetOmega;
	private double targetAlpha;
	private double targetHeading;
	
	private double kPv = 0.0; //0.0
	private double kA = 0.0; //0.0
	private double kP = 0.3;  //2.5
	private double kI = 0.0;  //0.0
	private double kD = 0.2;  //0.3
	
	private double kpv = 0.0; //0.5
	private double ka = 0.0; //0.0
	private double kp = 0.0; //10.0
	private double ki = 0.0; //0.0
	private double kd = 0.0; //0.5
	
	private double minLineOutput = 0.0; //0.085
	private double minTurnOutput = 0.3; //0.4

	private double dt = 0.005;
	
	private long currentTime;
	private long lastTime;
	private long lookupTime;
	private long startTime;
	
	public JoystickTrajectory joystickTraj;
	private StationaryTrajectory stationaryTraj;
	private TorPID displacementPID;
	private TorPID headingPID;
	
	protected static double displacementWaypoint;
	protected static double headingWaypoint;
	
	private boolean usingWaypoint = true;
	
	private TorMotionProfile(){
		joystickTraj = new JoystickTrajectory();
		stationaryTraj = new StationaryTrajectory();
		displacementPID = new TorPID(dt);
		headingPID = new TorPID(dt);
		
		defaultTrajectory = stationaryTraj;
		
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		
		displacementPID.setLimitMode(sensorLimitMode.Default);
		displacementPID.setNoiseMode(sensorNoiseMode.Noisy);
		displacementPID.setBacklash(0.0);
		displacementPID.setPositionTolerance(0.01);
		displacementPID.setVelocityTolerance(0.01);
		displacementPID.setMinimumOutput(minLineOutput);
		displacementPID.setkP(kP);
		displacementPID.setkI(kI);
		displacementPID.setkD(kD);
		displacementPID.setkPv(kPv);
		displacementPID.setkA(kA);
		
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
	
	public double lookUpDisplacement(long t){
		return activeTrajectory.lookUpDisplacement(lookupTime);
	}
	public double lookUpVelocity(long t){
		return activeTrajectory.lookUpVelocity(lookupTime);
	}
	public double lookUpAcceleration(long t){
		return activeTrajectory.lookUpAcceleration(lookupTime);
	}
	
	public double lookUpHeading(long t){
		
		return activeTrajectory.lookUpHeading(lookupTime);
	}
	public double lookUpOmega(long t){
		return activeTrajectory.lookUpOmega(lookupTime);
	}
	public double lookUpAlpha(long t){
		return activeTrajectory.lookUpAlpha(lookupTime);
	}
	
	public boolean lookUpIsLast(long t){
		return activeTrajectory.lookUpIsLast(lookupTime);
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
		TorCAN.INSTANCE.resetEncoder();
		TorCAN.INSTANCE.resetHeading();
		resetPID();
		isActive = true;
	}
	public void setInactive(){
		isActive = false;
	}
	public boolean isActive(){
		return isActive;
	}
	
	public double getTimeInterval(){
		return timeInterval;
	}
	
	public void run(){		
		if(isActive()){
			currentTime = System.currentTimeMillis();
			dt = (currentTime - lastTime) * 0.001;
			lastTime = currentTime; //TODO (1): uncomment and re-tune PID, if necessary
			currentTime = (currentTime - (currentTime % ((long)(getTimeInterval() * 1000))));
			lookupTime = currentTime - startTime;
			
			displacementPID.updateDt(dt);
			headingPID.updateDt(dt);
			
//			joystickTraj.updateDt(dt); //TODO (2): uncomment and see if this makes things better/worse after doing (1).
//			joystickTraj.updateVelocity();
//			joystickTraj.updateOmega();
			
			//Displacement
			displacementPID.updatePosition(TorCAN.INSTANCE.getDisplacement());
			displacementPID.updateVelocity(TorCAN.INSTANCE.getVelocity());

			targetDisplacement = lookUpDisplacement(currentTime) + displacementWaypoint;
			targetVelocity = lookUpVelocity(currentTime);
			targetAcceleration = lookUpAcceleration(currentTime);	

			displacementPID.updatePositionTarget(targetDisplacement);
			displacementPID.updateVelocityTarget(targetVelocity);
			displacementPID.updateAccelerationTarget(targetAcceleration);

			SmartDashboard.putNumber("targetVelocity", targetVelocity);
			SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
			SmartDashboard.putNumber("targetDisplacement", targetDisplacement);
			SmartDashboard.putNumber("currentVelocity", -displacementPID.velocity());
			SmartDashboard.putNumber("currentAcceleration", -displacementPID.acceleration());
			SmartDashboard.putNumber("currentDisplacement", -displacementPID.position());
			SmartDashboard.putNumber("dDispErrordt", -displacementPID.dErrodt());
			SmartDashboard.putNumber("displacementError", -displacementPID.error());

			//Heading
			headingPID.updatePosition(TorCAN.INSTANCE.getHeading());
			headingPID.updateVelocity(TorCAN.INSTANCE.getOmega());

			targetHeading = lookUpHeading(currentTime) + headingWaypoint;
			targetOmega = lookUpOmega(currentTime);
			targetAlpha = lookUpAlpha(currentTime);	

			headingPID.updatePositionTarget(targetHeading);
			headingPID.updateVelocityTarget(targetOmega);
			headingPID.updateAccelerationTarget(targetAlpha);	

			SmartDashboard.putNumber("targetOmega", targetOmega);
			SmartDashboard.putNumber("targetAlpha", targetAlpha);
			SmartDashboard.putNumber("targetHeading", targetHeading);
			SmartDashboard.putNumber("currentOmega", -headingPID.velocity());
			SmartDashboard.putNumber("currentAlpha", -headingPID.acceleration());
			SmartDashboard.putNumber("currentHeading", -headingPID.position());
			SmartDashboard.putNumber("dHeadErrordt", -headingWaypoint);
			SmartDashboard.putNumber("headingError", -headingPID.error());

			displacementPID.update();
			headingPID.update();
			TorCAN.INSTANCE.setTargets(displacementPID.output(), headingPID.output());
//			TorCAN.INSTANCE.setTargets(0.0, 0.0);
			if(lookUpIsLast(currentTime) && displacementPID.isOnTarget() && headingPID.isOnTarget()){
				startTime = currentTime;
				if(!(activeTrajectory == defaultTrajectory && nextTrajectory == defaultTrajectory)){
					if(usingWaypoint){
						displacementWaypoint += lookUpDisplacement(-1);
						headingWaypoint += lookUpHeading(-1);
					}
					System.out.println("IS ON TARGETTTTTTTTTTTTTTTTTTTTTTTT");
					activeTrajectory = nextTrajectory;
					nextTrajectory = defaultTrajectory;
				}
			}
		}
		else{
			TorCAN.INSTANCE.resetEncoder();
			TorCAN.INSTANCE.resetHeading();
		}
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
		return displacementPID.isOnTarget();
	}
	
	public boolean headOnTarget(){
		return headingPID.isOnTarget();
	}
	
	public boolean lookUpIsLast(){
		return lookUpIsLast(currentTime);
	}
	
	public void resetWaypoints(){
		displacementWaypoint = 0;
		headingWaypoint = 0;
	}
	
	public void resetPID(){
		headingPID.reset();
		displacementPID.reset();
	}
}

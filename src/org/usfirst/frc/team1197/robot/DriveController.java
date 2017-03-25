package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.TorPID.sensorLimitMode;
import org.usfirst.frc.team1197.robot.TorPID.sensorNoiseMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController {
	
	public final DriveHardware hardware;
	public final JoystickTrajectory joystickTraj;
	private final TorPID translationPID;
	private final TorPID rotationPID;

	private boolean enabled = true; // Safer to assume we're enabled
	private boolean motionProfilingActive = false; 
	protected boolean isHighGear;
	protected boolean usingCarDriveForHighGear = false;
	private TorTrajectory defaultTrajectory = null;
	public TorTrajectory activeTrajectory = null;
	public TorTrajectory nextTrajectory = null;
	
	private double targetVelocity;
	private double targetAcceleration;
	private double targetPosition;

	private double targetOmega;
	private double targetAlpha;
	private double targetHeading;

	private final double timeInterval = 0.005;
	private double dt = timeInterval;
	private long currentTime;
	private long lastTime;
	private long lookupTime;
	private long startTime;

	protected double positionWaypoint;
	protected double headingWaypoint;

	public DriveController() {
		hardware = new DriveHardware();
		joystickTraj = new JoystickTrajectory();
		translationPID = new TorPID(dt);
		rotationPID = new TorPID(dt);

		defaultTrajectory = joystickTraj;

		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		
		setClosedLoopConstants(Robot.mode);
		
		translationPID.setLimitMode(sensorLimitMode.Default);
		translationPID.setNoiseMode(sensorNoiseMode.Noisy);
		translationPID.setBacklash(0.0);
		translationPID.setPositionTolerance(0.05);
		translationPID.setVelocityTolerance(0.0125);

		rotationPID.setLimitMode(sensorLimitMode.Coterminal);
		rotationPID.setNoiseMode(sensorNoiseMode.Noisy);
		rotationPID.setBacklash(0.0);
		rotationPID.setPositionTolerance(0.0125);
		rotationPID.setVelocityTolerance(0.0125);
		
		resetPID();
		resetWaypoints();
		disable();
	}
	
	public void run() {
		currentTime = System.currentTimeMillis();
		dt = (currentTime - lastTime) * 0.001;
		lastTime = currentTime;
		currentTime = (currentTime - (currentTime % ((long) (timeInterval * 1000))));
		lookupTime = currentTime - startTime;
		translationPID.updateDt(dt);
		rotationPID.updateDt(dt);
		joystickTraj.updateDt(dt);

		// Translation
		translationPID.updatePosition(hardware.getPosition());
//		translationPID.updateVelocity(hardware.getVelocity());

		// Rotation
		rotationPID.updatePosition(hardware.getHeading());
		rotationPID.updateVelocity(hardware.getOmega());

		if (motionProfilingActive) {
			if (activeTrajectory == joystickTraj) {
				joystickTraj.updateVelocity();
				joystickTraj.updateOmega();
			}
		} else {
			joystickTraj.setState(translationPID.position(), translationPID.velocity(),
					rotationPID.position(), rotationPID.velocity());
		}

		targetPosition = activeTrajectory.lookUpPosition(lookupTime);
		if (activeTrajectory != joystickTraj) {
			targetPosition += positionWaypoint;
		}
		targetVelocity = activeTrajectory.lookUpVelocity(lookupTime);
		targetAcceleration = activeTrajectory.lookUpAcceleration(lookupTime);

		translationPID.updatePositionTarget(targetPosition);
		translationPID.updateVelocityTarget(targetVelocity);
		translationPID.updateAccelerationTarget(targetAcceleration);

		targetHeading = activeTrajectory.lookUpHeading(lookupTime);
		if (activeTrajectory != joystickTraj) {
			targetHeading += headingWaypoint;
		}
		targetOmega = activeTrajectory.lookUpOmega(lookupTime);
		targetAlpha = activeTrajectory.lookUpAlpha(lookupTime);

		rotationPID.updatePositionTarget(targetHeading);
		rotationPID.updateVelocityTarget(targetOmega);
		rotationPID.updateAccelerationTarget(targetAlpha);

		translationPID.update();
		rotationPID.update();
		
		SmartDashboard.putNumber("Rotation Error", rotationPID.error());

		if (motionProfilingActive) {
			if (activeTrajectory != joystickTraj) {
				joystickTraj.setState(targetPosition, targetVelocity, targetHeading, targetOmega);
			}
			graphTranslation();
			graphRotation();
			hardware.setTargets(translationPID.output(), rotationPID.output());
		}
		if (activeTrajectory.lookUpIsLast(lookupTime) && translationPID.isOnTarget() && rotationPID.isOnTarget()) {
			startTime = currentTime;
			if (!(activeTrajectory == defaultTrajectory && nextTrajectory == defaultTrajectory)) {
				positionWaypoint = targetPosition;
				headingWaypoint = targetHeading;
				activeTrajectory.setComplete(true);
				nextTrajectory.setComplete(false);
				activeTrajectory = nextTrajectory;
				nextTrajectory = defaultTrajectory;
			}
		}
	}
	
	/**
	 * @param state
	 */
	public void setClosedLoopConstants(RobotMode state){
		if(state == RobotMode.AUTO){
			rotationPID.setMinimumOutput(0.0);
			rotationPID.setkP(22.0);
			rotationPID.setkI(10.0);
			rotationPID.setkD(0.5);
			rotationPID.setkPv(0.0);
			rotationPID.setkA(0.0);
			translationPID.setMinimumOutput(0.0);
			translationPID.setkP(17.0);
			translationPID.setkI(1.0);
			translationPID.setkD(0.5);
			translationPID.setkPv(0.0);
			translationPID.setkA(0.0);
			
			translationPID.setPositionTolerance(0.05);
			translationPID.setVelocityTolerance(0.0125);
			rotationPID.setPositionTolerance(0.0125);
			rotationPID.setVelocityTolerance(0.0125);
		}
		else if(state == RobotMode.TELEOP){
//			rotationPID.setMinimumOutput(0.0);
//			rotationPID.setkP(5.0);
//			rotationPID.setkI(0.0);
//			rotationPID.setkD(0.1);
//			rotationPID.setkPv(0.0);
//			rotationPID.setkA(0.0);
//			translationPID.setMinimumOutput(0.0);
//			translationPID.setkP(5.0);
//			translationPID.setkI(0.0);
//			translationPID.setkD(0.1);
//			translationPID.setkPv(0.0);
//			translationPID.setkA(0.0);
//			
//			translationPID.setPositionTolerance(0.05);
//			translationPID.setVelocityTolerance(0.0125);
//			rotationPID.setPositionTolerance(0.0125);
//			rotationPID.setVelocityTolerance(0.0125);
			
			rotationPID.setMinimumOutput(0.0);
			rotationPID.setkP(0.1);
			rotationPID.setkI(0.0);
			rotationPID.setkD(0.0);
			rotationPID.setkPv(0.0);
			rotationPID.setkA(0.0);
			translationPID.setMinimumOutput(0.0);
			translationPID.setkP(0.1);
			translationPID.setkI(0.0);
			translationPID.setkD(0.0);
			translationPID.setkPv(0.0);
			translationPID.setkA(0.0);
			
			translationPID.setPositionTolerance(0.1);
			translationPID.setVelocityTolerance(0.125);
			rotationPID.setPositionTolerance(0.15);
			rotationPID.setVelocityTolerance(1.25);
		}
		else{
			rotationPID.setMinimumOutput(0.0);
			rotationPID.setkP(0.0);
			rotationPID.setkI(0.0);
			rotationPID.setkD(0.0);
			rotationPID.setkPv(0.0);
			rotationPID.setkA(0.0);
			translationPID.setMinimumOutput(0.0);
			translationPID.setkP(0.0);
			translationPID.setkI(0.0);
			translationPID.setkD(0.0);
			translationPID.setkPv(0.0);
			translationPID.setkA(0.0);
			
			translationPID.setPositionTolerance(0.5);
			translationPID.setVelocityTolerance(0.125);
			rotationPID.setPositionTolerance(1.25);
			rotationPID.setVelocityTolerance(1.25);
		}
	}

	public void loadTrajectory(TorTrajectory traj) {
		nextTrajectory = traj;
		traj.setComplete(false);
	}
	
	public void setTargets(double a, double b) {
		if (motionProfilingActive) {
			if(activeTrajectory == joystickTraj) {
				joystickTraj.setTargets(a, b);
			}
		} else {
			hardware.setMotorSpeeds(a, b);
		}
	}

	public void setMotionProfilingActive() {
		motionProfilingActive = true;
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		resetWaypoints(); // TODO: remove
		resetPID();
		hardware.chooseVelocityControl();
	}

	public void setMotionProfilingInactive() {
		motionProfilingActive = false;
		activeTrajectory = defaultTrajectory;
		nextTrajectory = defaultTrajectory;
		hardware.choosePercentVbus();
	}

	public boolean motionProfilingActive() {
		return motionProfilingActive;
	}

	// Shifts the robot to high gear and sets motion profiling to active.
	public void shiftToHighGear(){
		if (!isHighGear){
			isHighGear = true;
			hardware.shiftToHighGear();
			if (motionProfilingActive) {
				System.err.println("Error: expected inactive motion profiling before transition to high gear.");
				// If this happens, something went wrong.
			} else {
				if (!usingCarDriveForHighGear && Robot.mode == RobotMode.TELEOP) {
					// If we're in teleop but we're not supposed to be using
					// carDrive for high gear, don't turn on motion profiling.
				} else {
					setMotionProfilingActive();
				}
			}
		}
	}
	
	// Shifts the robot to low gear and sets motion profiling to inactive.
	public void shiftToLowGear(){
		if (isHighGear){
			isHighGear = false;
			hardware.shiftToLowGear();
			if (motionProfilingActive) {
				setMotionProfilingInactive();
			}
		}
	}
	
	public void useCarDriveInHighGear(boolean b){
		usingCarDriveForHighGear = b;
		if (usingCarDriveForHighGear 
				&& Robot.mode == RobotMode.TELEOP 
				&& isHighGear 
				&& !motionProfilingActive) {
			setMotionProfilingActive();
		} else if (!usingCarDriveForHighGear 
				&& Robot.mode == RobotMode.TELEOP 
				&& isHighGear 
				&& motionProfilingActive) {
			setMotionProfilingInactive();
		}
	}
	
	public void enable() {
		if (enabled) {
			disable(); //Guarantee safe transition between AUTO/TELEOP/TEST.
		}
//		resetWaypoints(); //TODO: Uncomment
		enabled  = true;
		isHighGear = false;
		shiftToHighGear();
	}

	public void disable() {
		enabled = false;
		if (motionProfilingActive) {
			setMotionProfilingInactive();
		}
		hardware.shiftToHighGear(); // Stay in high gear mechanically since that's the default.
		hardware.setMotorSpeeds(0.0, 0.0);
	}

	public void graphTranslation() {
		SmartDashboard.putNumber("targetVelocity", targetVelocity);
		SmartDashboard.putNumber("targetAcceleration", targetAcceleration);
		SmartDashboard.putNumber("targetPosition", targetPosition);
		SmartDashboard.putNumber("currentVelocity", translationPID.velocity());
		SmartDashboard.putNumber("currentAcceleration", translationPID.acceleration());
		SmartDashboard.putNumber("currentPosition", translationPID.position());
		SmartDashboard.putNumber("dDispErrordt", translationPID.dErrodt());
		SmartDashboard.putNumber("positionError", translationPID.error());
	}

	public void graphRotation() {
		SmartDashboard.putNumber("targetOmega", targetOmega);
		SmartDashboard.putNumber("targetAlpha", targetAlpha);
		SmartDashboard.putNumber("targetHeading", targetHeading);
		SmartDashboard.putNumber("currentOmega", rotationPID.velocity());
		SmartDashboard.putNumber("currentAlpha", rotationPID.acceleration());
		SmartDashboard.putNumber("currentHeading", rotationPID.position());
		SmartDashboard.putNumber("dHeadErrordt", headingWaypoint);
		SmartDashboard.putNumber("headingError", rotationPID.error());
	}

	public boolean onTargetTranslation() {
		return translationPID.isOnTarget();
	}

	public boolean onTargetRotation() {
		return rotationPID.isOnTarget();
	}
	
	public double getPositionError(){
		return translationPID.error();
	}
	public double getVelocityError(){
		return translationPID.velocity();
	}
	public double getHeadingError(){
		return rotationPID.error();
	}
	public double getPositionTolerance(){
		return translationPID.positionTolerance();
	}
	public double getVelocityTolerance(){
		return translationPID.velocityTolerance();
	}
	public double getHeadingTolerance(){
		return rotationPID.positionTolerance();
	}

	public void resetWaypoints() {
//		hardware.resetEncoder(); // TODO: Uncomment
//		hardware.resetGyro();
//		positionWaypoint = 0.0;
//		headingWaypoint = 0.0;
		positionWaypoint = translationPID.position(); // TODO: remove
		headingWaypoint = rotationPID.position();
	}

	public void resetPID() {
		rotationPID.reset();
		translationPID.reset();
	}
}

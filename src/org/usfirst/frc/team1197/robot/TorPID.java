package org.usfirst.frc.team1197.robot;

public class TorPID {
	private double v;
	private double kP;
	private double kI;
	private double kD;
	private double kPv;
	private double kA;
	
	private double posTolerance;
	private double velTolerance;
	private double minimumOutput;
	
	private double pos;
	private double last_pos;
	private double vel;
	private double last_vel;
	private double acc;
	private TorDerivative PositionDerivative;
	private TorDerivative ErrorDerivative;
	private double error;
	private double last_error;
	private double dErrordt;
	private double errorIntegral;
	private double timeInterval;
	private double dt;
	private boolean dtIsUpdated;
	private boolean velIsUpdated;
	
	public enum sensorLimitMode{Default, Coterminal;}
	public enum sensorNoiseMode{Clean, Noisy;}
	public enum backlashMode{Tight, Sloppy;}
	private sensorLimitMode limitMode = sensorLimitMode.Default;
	private sensorNoiseMode noiseMode = sensorNoiseMode.Clean;
	private backlashMode backlashStatus = backlashMode.Tight;
	private double backlash = 0.0;
	
	private double posTarget;
	private double velTarget;
	private double accTarget;
	
	public TorPID(double timeInterval){
		this.timeInterval = timeInterval;
		ErrorDerivative = new TorDerivative(timeInterval);
		PositionDerivative = new TorDerivative(timeInterval);
	}
	
	public void update(){
		// Set dt to a safe value, if it hasn't been updated:
		if (!dtIsUpdated || dt == 0.0)
			dt = timeInterval;
		dtIsUpdated = false;
		
		// Update velocity, if necessary:
		if (!velIsUpdated){
			// TODO: think about how to guarantee continuous velocity w/ coterminal sensor limits.
			if(noiseMode == sensorNoiseMode.Noisy){
				vel = PositionDerivative.estimate(pos);
				acc = PositionDerivative.second();
			}
			else{
				vel = (pos-last_pos)/dt;
				acc = (vel-last_vel)/dt;
			}
		}
		else{
			acc = (vel-last_vel)/dt;
		}
		last_pos = pos;
		last_vel = vel;
		velIsUpdated = false;
		
		// If we have to compensate for backlash, do it here:
		if(backlashStatus == backlashMode.Sloppy){
			if(accTarget > 0)
				pos -= (0.5 * backlash);
			else if(accTarget < 0)
				pos += (0.5 * backlash);
			else{
				if(Math.abs(posTarget - pos) < posTolerance)
					pos = posTarget;
			}
		}
		
		// calculate error, integral, and derivative:
		error = posTarget - pos;
		if(limitMode == sensorLimitMode.Coterminal) {
			while(error < -Math.PI) error += 2*Math.PI;
			while(error > Math.PI) error -= 2*Math.PI;
		}
		// Only continue to integrate if we're outside our tolerance:
		if(Math.abs(error) > posTolerance){
			errorIntegral += error*dt;
		}
		// Don't use the I term if we're not supposed to be moving:
		if(Math.abs(velTarget) <= velTolerance){
			errorIntegral = 0.0;
		}
		// TODO: think about how to guarantee continuous velocity w/ coterminal sensor limits.
		if(noiseMode == sensorNoiseMode.Noisy)
			dErrordt = ErrorDerivative.estimate(error);
		else
			dErrordt = (error-last_error)/dt;
		last_error = error;
		
		// Finally, calculate output:
		v = velTarget
				+ (kPv * (velTarget-vel))
				+ (kA * accTarget )
				+ (kP * error) 
				+ (kI * errorIntegral)
				+ (kD * dErrordt);
		if(Math.abs(velTarget) > 0.0 || error > posTolerance){
			if(v < 0.0)
				v -= minimumOutput;
			else if(v > 0.0)
				v += minimumOutput;
		}
	}
	
	// update() and output() are separated to facilitate updates to
	// velocity, error derivative, etc even when output is not needed.
	public double output(){
		return v;
	}
	
	public boolean isOnTarget(){
//		SmartDashboard.putNumber("Math.abs(vel)", Math.abs(vel));
//		SmartDashboard.putNumber("velTarget", velTarget);
//		SmartDashboard.putNumber("Math.abs(error)", Math.abs(error));
		return ( (Math.abs(error) <= posTolerance) 
			  && (Math.abs(velTarget) <= velTolerance) 
			  && (Math.abs(vel) <= velTolerance) );
	}
	
	// SETUP METHODS (recommended: call all of them!)
	public void setLimitMode(sensorLimitMode mode){
		this.limitMode = mode;
	}
	
	public void setNoiseMode(sensorNoiseMode mode){
		this.noiseMode = mode;
	}
	
	public void setBacklash(double backlash){
		this.backlash = Math.abs(backlash);
		if (this.backlash == 0.0){
			backlashStatus = backlashMode.Tight;
		}
		else{
			backlashStatus = backlashMode.Sloppy;
			posTolerance = 0.5*backlash;
		}
	}
	
	public void setPositionTolerance(double t){
		//TODO: if backlash is already nonzero, we should probably fail a debug assertion
		posTolerance = t;
	}
	
	public void setVelocityTolerance(double t){
		velTolerance = t;
	}
	
	public void setMinimumOutput(double min){
		this.minimumOutput = min;
	}
	
	public void setkP(double P){
		this.kP = P;
	}
	
	public void setkI(double I){
		this.kI = I;
	}
	
	public void setkD(double D){
		this.kD = D;
	}
	
	public void setkPv(double Pv){
		this.kPv = Pv;
	}
	
	public void setkA(double A){
		this.kA = A;
	}
	
	// UPDATE METHODS: call these once per iteration (where necessary)
	public void updateDt(double dt){
		this.dt = dt;
		dtIsUpdated = true;
	}
	
	public void updatePosition(double pos){
		this.pos = pos;
	}
	
	// Call updateVelocity() if the sensor provides a clean velocity
	// reading natively, e.g. like a gyro. If updateVelocity() is not
	// called, TorPID will calculate velocity automatically.
	public void updateVelocity(double vel){
		this.vel = vel;
		velIsUpdated = true;
	}
	
	public void updatePositionTarget(double pos){
		this.posTarget = pos;
	}
	
	public void updateVelocityTarget(double vel){
		this.velTarget = vel;
	}
	
	public void updateAccelerationTarget(double acc){
		this.accTarget = acc;
	}
	
	// ACCESSOR METHODS - useful for printouts etc.
	public double error(){
		return error;
	}
	
	public double errorIntegral(){
		return errorIntegral;
	}
	
	public double dErrodt(){
		return dErrordt;
	}
	
	public double position(){
		return pos;
	}
	
	public double velocity(){
		return vel;
	}
	
	public double acceleration(){
		return acc;
	}
	
	public double positionTolerance(){
		return posTolerance;
	}
	
	public double velocityTolerance(){
		return velTolerance;
	}
	
	public void reset(){
//		PositionDerivative.reset();
		errorIntegral = 0;
	}
}
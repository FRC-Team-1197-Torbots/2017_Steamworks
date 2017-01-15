package org.usfirst.frc.team1197.robot;

public class StationaryTrajectory extends TorTrajectory{
	
	public StationaryTrajectory() {
//		super(0);
		type = new String("Stationary");
	}
	
	public boolean lookUpIsLast(long t){
		return true;
	}
	
	public double lookUpDisplacement(long t){
		return goal_pos;
	}
	public double lookUpVelocity(long t){
		return 0.0;
	}
	public double lookUpAcceleration(long t){
		return 0.0;
	}
	
	public double lookUpHeading(long t){
		return 0;
	}
	public double lookUpOmega(long t){
		return 0;
	}
	public double lookUpAlpha(long t){
		return 0;
	}
	
	public void execute(){
		TorMotionProfile.INSTANCE.loadTrajectory(this);
	}

}

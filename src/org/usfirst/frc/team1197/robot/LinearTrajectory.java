package org.usfirst.frc.team1197.robot;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal, 0.0);
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
	}
	
	public double lookUpOmega(long t){
		return 0;
	}
	public double lookUpAlpha(long t){
		return 0;
	}
	public double lookUpHeading(long t){
		return 0;
	}
}
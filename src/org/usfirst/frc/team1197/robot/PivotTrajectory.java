package org.usfirst.frc.team1197.robot;

public class PivotTrajectory extends TorTrajectory {
	
	public PivotTrajectory(double goal){
		super(0.0, goal * (Math.PI/180.0));
		build(goal_head, max_omg, max_alf, max_jeta, rotation);
	}
	
	public double lookUpHeading(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return goal_head;
		}
		return rotation.get(i).pos;
	}
	public double lookUpOmega(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return rotation.get(i).vel;
	}
	public double lookUpAlpha(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return rotation.get(i).acc;
	}
	
	public double lookUpPosition(long t){
		return 0;
	}
	public double lookUpVelocity(long t){
		return 0;
	}
	public double lookUpAcceleration(long t){
		return 0;
	}
}
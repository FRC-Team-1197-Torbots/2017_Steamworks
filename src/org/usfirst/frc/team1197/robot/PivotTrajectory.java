package org.usfirst.frc.team1197.robot;

public class PivotTrajectory extends TorTrajectory {
	
	public PivotTrajectory(double goal){
		super(0.0, goal * (Math.PI/180.0));
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_head, max_omg, max_alf, max_jeta, rotation);
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
	
	public String toString(){
		return "Pivot Trajectory \n" +
				   "GoalHead: " + goal_head + "\n";
	}
}
package org.usfirst.frc.team1197.robot;

import java.util.LinkedList;

import java.util.List;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal);
		type = new String("Linear");
		build(goal_pos, max_vel, max_acc, max_jerk, displacement, velocity, acceleration);
	}
	
	public double lookUpDisplacement(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return goal_pos;
		}
		return displacement.elementAt(i);
	}
	public double lookUpVelocity(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return velocity.elementAt(i);
	}
	public double lookUpAcceleration(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return acceleration.elementAt(i);
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
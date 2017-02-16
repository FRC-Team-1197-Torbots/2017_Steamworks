package org.usfirst.frc.team1197.robot;

public class MotionState1D {
	public double pos;
	public double vel;
	public double acc;
	public MotionState1D(){
		pos = 0.0;
		vel = 0.0;
		acc = 0.0;
	}
	public MotionState1D(double p, double v, double a){
		pos = p;
		vel = v;
		acc = a;
	}
	public void set(double p, double v, double a){
		pos = p;
		vel = v;
		acc = a;
	}
}

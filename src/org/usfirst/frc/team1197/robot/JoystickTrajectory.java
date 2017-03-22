package org.usfirst.frc.team1197.robot;

public class JoystickTrajectory extends TorTrajectory{
	
	private Motion linearMotion;
	private Motion rotationalMotion;
	private double tgt_vel;
	private double tgt_omg;
	private double tgt_acc;
	
	public JoystickTrajectory(){
		super(0.0, 0.0);
		max_alf = 11.0;
		max_acc = 2.75;
		linearMotion = new Motion(0.0, 0.0, 0.0);
		rotationalMotion = new Motion(0.0, 0.0, 0.0);
	}
	
	public double goalPos(){
		return linearMotion.pos;
	}
	public double goalHead(){
		return rotationalMotion.pos;
	}
	
	public double lookUpPosition(long t){
		return linearMotion.pos;
	}
	public double lookUpVelocity(long t){
		return linearMotion.vel;
	}
	public double lookUpAcceleration(long t){
		return linearMotion.acc;
	}
	
	public double lookUpHeading(long t){
		return rotationalMotion.pos;
	}
	public double lookUpOmega(long t){
		return rotationalMotion.vel;
	}
	public double lookUpAlpha(long t){
		return rotationalMotion.acc;
	}
	
	public boolean lookUpIsLast(long t){
		return true;
	}
	
	public void setTargets(double v, double w){
		tgt_vel = v;
		tgt_omg = w;
	}
	
	public void setState(double pos, double v, double head, double w){
		linearMotion.pos = pos;
		linearMotion.vel = v;
		tgt_vel = v;
		rotationalMotion.pos = head;
		rotationalMotion.vel = w;
		tgt_omg = w;
	}
	
	public void update(double tgt_vel, Motion m, double max_acc){
		// Target (requested) acceleration:
		tgt_acc = (tgt_vel - m.vel) / dt;
		// Actual acceleration:
		m.acc = Math.signum(tgt_acc)*Math.min(Math.abs(tgt_acc), max_acc);
		// Velocity:
		m.vel = m.vel + m.acc*dt;
		// Position:
		m.pos = m.pos + m.vel*dt + 0.5*m.acc*dt*dt;	
	}
	
	public void updateVelocity(){
		update(tgt_vel, linearMotion, max_acc);
	}
	
	public void updateOmega(){
		update(tgt_omg, rotationalMotion, max_alf);
	}
	
	public void updateDt(double dt){
		if(dt == 0.0){
			this.dt = 0.005;
		}
		else{
			this.dt = dt;
		}
	}
	
	public String toString(){
		return "Joystick Trajectory \n" +
			   "GoalPos: " + goal_pos + "\n" +
			   "GoalHead: " + goal_head + "\n";
	}
}

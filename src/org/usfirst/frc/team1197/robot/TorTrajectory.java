
package org.usfirst.frc.team1197.robot;

import java.util.Vector;
import java.util.LinkedList;
import java.util.List;

public abstract class TorTrajectory {
	protected double goal_pos = 0.0;
	protected double goal_head = 0.0;
	
	protected double max_vel = 1000.0;
	protected double max_acc = 1000.0;
	protected double max_jerk = 1000.0;
	
	protected double max_omg;
	protected double max_alf;
	protected double max_jeta;
	
	protected Vector<Long> time;
	
	protected Vector<Double> position;
	protected Vector<Double> velocity;
	protected Vector<Double> acceleration;
	
	protected Vector<Double> omega;
	protected Vector<Double> alpha;
	protected Vector<Double> heading;
	
	protected static long startTime;
	protected double dt = 0.005;
	
	public class Motion {
		public double pos;
		public double vel;
		public double acc;
		
		public Motion(double p, double v, double a){
			pos = p;
			vel = v;
			acc = a;
		}
	}
	
	public TorTrajectory(double goal_pos, double goal_head){
		this.goal_pos = goal_pos;
		this.goal_head = goal_head;
		
		max_vel = 15.0; //2.5
		max_acc = 17.0; //6.0 
		max_jerk = 29.0; //12.0
		
		max_omg = 30.0; //14.0
		max_alf = 32.0; //16.0
		max_jeta = 38.0; //22.0
		
		time = new Vector<Long>();
		
		position = new Vector<Double>();
		velocity = new Vector<Double>();
		acceleration = new Vector<Double>();
		
		omega = new Vector<Double>();
		alpha = new Vector<Double>();
		heading = new Vector<Double>();
	}
	
	public TorTrajectory(){
		
	}
	
	// The following magic was adapted from 254's TrajectoryLib.
	protected void build(double goal_pos, double max_vel, double max_acc, double max_jerk, 
						Vector<Double> position, Vector<Double> velocity, Vector<Double> acceleration){
		position.clear();
		velocity.clear();
		acceleration.clear();
		time.clear();
		// This guarantees that if we don't have the runway to accelerate up to top speed
		// or to "jerk" up to top acceleration, we set more realistic goals:
		
		double disp = Math.abs(goal_pos); // Worry about sign later.
		double adjusted_max_acc = Math.min(max_acc,
										   Math.min(Math.sqrt(max_vel * max_jerk),
										            Math.pow((0.5 * disp * max_jerk * max_jerk), (1.0/3.0))));
		double adjusted_max_vel = Math.min( max_vel,
				(-adjusted_max_acc * adjusted_max_acc + Math.sqrt(adjusted_max_acc
						* adjusted_max_acc * adjusted_max_acc * adjusted_max_acc
						+ 4 * max_jerk * max_jerk * adjusted_max_acc * disp)) 
				/ (2 * max_jerk) );
		
		// Since each filter has a discrete number of slots, we need this "fancy rounding code" 
		// to make sure the robot always drives exactly the correct distance, and always
		// moves in a way that does not exceed the maximum velocity, acceleration, or
		// jerk. Without this, the use of a digital filter to produce motion profiles can
		// give unexpected results that violate these constraints, especially when
		// the path is short or the maximum velocity, acceleration, and/or jerk are high.

		int f0_length = (int) (1 + Math.round(disp / (adjusted_max_vel * dt)));
		adjusted_max_vel = disp/(f0_length * dt);
		int f1_length = (int) Math.ceil(adjusted_max_vel / (adjusted_max_acc * dt));
		int f2_length = (int) Math.ceil(adjusted_max_acc / (max_jerk * dt));
		int tot_length = f0_length + f1_length + f2_length;
		if(goal_pos < 0.0){
			adjusted_max_vel = -adjusted_max_vel;
		}
		secondOrderFilter(f0_length, f1_length, f2_length, dt, adjusted_max_vel, tot_length, position, velocity, acceleration);
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_vel, int tot_length,
			Vector<Double> position, Vector<Double> velocity, Vector<Double> acceleration) {
		// Why no "f0"? Because the zero-filter can be equivalently implemented more
		// simply by just feeding a constant velocity value into the first filter for
		// the correct length of time. The "real" filters, on the other hand, MUST be 
		// implemented as lists of numbers:
		List<Double> f1 = new LinkedList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
		List<Double> f2 = new LinkedList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		// Inputs and outputs of the filters:
		double input;
		double FL1out;
		double FL2out;
		// Individual data values:
		double pos = 0.0;
		double vel = 0.0;
		double acc = 0.0;
		long t = 0;
		// record previous values so we can do integration/differentiation:
		double last_pos = 0.0;
		double last_vel = 0.0;
		
		for (int i = 0; i < tot_length; ++i) {
			last_pos = pos;
			last_vel = vel;
			// As long as the zero filter is not empty, feed-forward the target velocity:
			if(f0_length > 0){
				input = max_vel;
				f0_length--; // Decrement the zero filter.
			}
			else{
				input = 0.0;
			}

			// Input goes at the beginning of the filter (end of the list):
			f1.add(new Double(input));
			// Throw away the element at the end of the filter (beginning of the list)
			f1.remove(0);
			//  Output is the average of the elements in the first filter:
			FL1out = average(f1);
			// Repeat this procedure for the second filter:
			f2.add(new Double(FL1out));
			f2.remove(0);
			FL2out = average(f2);
			
			// The output of the filter is velocity:
			vel = FL2out;
			velocity.addElement(new Double(vel));
		    // We have to integrate to get position. This uses trapezoidal integration,
		    // but the choice of integration strategy probably doesn't matter:
			pos = last_pos + 0.5 * (last_vel + vel) * dt;
			position.addElement(new Double(pos));
			// We have to differentiate to get acceleration:
			acc = (vel - last_vel) / dt;
			acceleration.addElement(new Double(acc));
			t += (long)(dt * 1000);
			time.addElement(new Long(t));
		}
	}
	
	public void execute(){
		startTime = System.currentTimeMillis();
		startTime = startTime - (startTime % 10);
		TorMotionProfile.INSTANCE.loadTrajectory(this);
	}
	
	public double goalPos(){
		return goal_pos;
	}
	
	public abstract double lookUpPosition(long t);
	public abstract double lookUpVelocity(long t);
	public abstract double lookUpAcceleration(long t);
	
	public abstract double lookUpAlpha(long t);
	public abstract double lookUpOmega(long t);
	public abstract double lookUpHeading(long t);	
	
	public boolean lookUpIsLast(long t){
		if(t < time.get(0)){
			return false;
		}
		int i = time.indexOf(t);
		if(i == -1){
			return true;
		}
		return (i+1 == time.size());
	}
	
	public double average(List<Double> list){
		double avg = 0;
		for(Double element : list){
			avg += element;
		}
		avg /= list.size();
		return avg;
	}
}

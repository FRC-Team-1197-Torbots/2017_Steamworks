package org.usfirst.frc.team1197.robot;

public class TorJoystickProfiles {
	
	private double k = 0.1; //0.3
	private double A = (k * k) / (1 - (2 * k));
	private double B = (k - 1) * (k - 1) / (k * k);	
	
	private double maxTurnRadius = 10.0;
	private double minTurnRadius = 0.5;
	private double steeringDeadBand = 0.1;
	private double throttleDeadBand = 0.15;
	private double C = (Math.log(minTurnRadius / maxTurnRadius)) / (steeringDeadBand - 1);
	private double D = maxTurnRadius * Math.exp(C * steeringDeadBand);
	private double negSteeringInertia = 0.0;
	private double previous_r = 0.0;
	private double r = 0.0;
	private double negThrottleInertia = 0.0;
	private double previous_y = 0.0;
	private double y = 0.0;
	
	public TorJoystickProfiles(){
		
	}
	
	public double findRadiusExponential(double x){
		/* See https://www.desmos.com/calculator/g0st5i5ajy 
		   for the turning profile equations we've played with. */
		r = Math.signum(x) * (D * Math.exp(-C * Math.abs(x)));

		/* Calculating the negative Steering Inertia.
		   See the negative inertia section
		   http://frc971.org/content/programming. */
		negSteeringInertia = (r - previous_r) * (1000 * 0.005);
		previous_r = r;
		if(x == 0.0)
			return 0.0;
		else{
			//Adding the negative Steering Inertia to prevent the "sluggish" steering. 
			return r + negSteeringInertia;
		}
	}
	
	public double findSpeed(double x){
		/* See https://www.desmos.com/calculator/0sixb0kl2b
		   for the throttle profile equations we've played with. */
		y = Math.signum(x) * A * ((Math.pow(B, Math.abs(x))) - 1.0);
		negThrottleInertia = (y - previous_y) * (1000 * 0.005);
		previous_y = y;
		if(Math.abs(x) >= throttleDeadBand)
			return y + negThrottleInertia;
		else
			return 0.0;
	}
	
	// "Simple" means no negative inertia.
	public double findRadiusSimple(double x){
		/* See https://www.desmos.com/calculator/g0st5i5ajy 
		   for the turning profile equations we've played with. */
		r = Math.signum(x) * (D * Math.exp(-C * Math.abs(x)));
		if(x == 0.0)
			return 0.0; // note: we define "zero radius" as driving straight!
		else
			return r; // output ranges from minTurnRadius to maxTurnRadius
	}
	
	// "Simple" means no negative inertia.
	public double findSpeedSimple(double x){
		/* See https://www.desmos.com/calculator/0sixb0kl2b
		   for the throttle profile equations we've played with. */
		y = Math.signum(x) * A * ((Math.pow(B, Math.abs(x))) - 1.0);
		if(Math.abs(x) >= throttleDeadBand)
			return y; // output ranges from 0.0 to 1.0
		else
			return 0.0;
	}
	
	// Accessor method for other classes to see the value of minTurnRadius.
	public double getMinTurnRadius(){
		return minTurnRadius;
	}
	
	// Accessor method for other classes to see the value of maxTurnRadius.
	public double getMaxTurnRadius(){
		return maxTurnRadius;
	}

}

package org.usfirst.frc.team1197.robot;

// **********************************************************************************************
// This class lets us take the derivative of noisy signals. This is very useful for determining
// sensor speed for noisy analog sensors like potentiometers or for coarse incremental encoders.
// Currently, the estimate() method must be called at regular intervals (dt) for valid results.
// Under the hood, this uses a least-squares best fit formula to calculate slope, and compensates
// for delay using a simple kinematic approximation. 
// **********************************************************************************************

public class TorDerivative {
	// Arrays to keep track of variable history:
	private double[] yData = new double[20];
	private double[] vData = new double[20];
	
	// SS stands for "sum of squares" (see MathWorld link below)
	private double SStt;
	private double SSty;
	private double SStv;

	private double dt;
	private double tMean;
	private double v0;
	private double a0;
	private double v;
	
	public TorDerivative(double dt){
		this.dt = dt;
		tMean = (yData.length-1) * dt * 0.5;
		// Note the symmetry between this and the SStx() method:
		SStt = 0;
	    for(int i = 0;  i < yData.length; i++){
			SStt += (i*dt - tMean)*(i*dt - tMean);
		}
	}
	
	private double Mean(double[] data){
		double sum = 0;
		for(int i=0; i < data.length; i++){
			sum += data[i];
		}
		double mean = sum / data.length;
		return mean;
	}
	
	// Push() appends 'x' to the beginning of the array 'data', moves
	// all elements forward, and discards the last element.
	private void Push(double[] data, double x){
		for(int i=data.length-1; i>0; i--){
			data[i] = data[i - 1];
		}
		data[0] = x;
	}
	
	private double SStx(double[] data){
		double mean  = Mean(data);
		double sum = 0.0;
	    for(int i=0; i<data.length; i++){
			sum += (i*dt - tMean)*(data[i] - mean);
		}
	    return sum;
	}
	
	public double estimate(double y){
		// This uses a least-squares formula to calculate best-fit slope.
		// If you're not familiar with this technique, you can learn more
		// here: http://mathworld.wolfram.com/LeastSquaresFitting.html
		Push(yData, y);
		SSty = SStx(yData);
		v0 = SSty / SStt;
		// Use another least-squares fit to find the "slope of the slope".
		// (This is also the second derivative of y, or acceleration!):
		Push(vData, v0);
		SStv = SStx(vData);
		a0 = SStv / SStt;
		// v0 alone is actually the *average* velocity of 'y' over the last
		// 'yData.length' iterations. Often, this will be roughly equal to the
		// the *median* velocity over that time frame. If we assume the velocity
		// changes linearly with time (constant acceleration), then this means
		// v0 is actually the velocity tMean seconds ago! To estimate current
		// velocity, we just add the product (acceleration x tMean) to our
		// original estimate of velocity (v0). This is the "simple kinematic
		// approximation" described in this class's header:
		v = v0 + a0*(tMean);
		return v;
	}
	
	// second() lets us query the 2nd derivative, since we have it: 
	public double second(){
		return a0;
	}
	
	// For symmetry:
	public double first(){
		return v;
	}
	
	public void reset(){
		for(int i = 0; i < yData.length; i++){
			yData[i] = 0.0;
			vData[i] = 0.0;
		}
	}
	
}

package org.usfirst.frc.team1197.robot;

// **********************************************************************************************
// This class lets us take the derivative of noisy signals. This is very useful for determining
// sensor speed for noisy analog sensors like potentiometers or for coarse incremental encoders.
// Currently, the estimate() method must be called at regular intervals (dt) for valid results.
// Under the hood, this uses a least squares best-fit formula to calculate slope. If you're not
// familiar with this technique, you can learn more here:
// 						http://mathworld.wolfram.com/LeastSquaresFitting.html
// **********************************************************************************************

public class TorDerivative {
	// Array to keep track of variable history:
	private double[] yData = new double[20];
	
	// SS stands for "sum of squares" (see MathWorld link above)
	private double SStt;
	private double SSty;

	private double dt;
	private double tMean;
	private double v0;
	private double v;
	private double a;
	
	public TorDerivative(double dt){
		this.dt = dt;
		reset();
		tMean = (yData.length-1) * dt * 0.5;
		// Note the symmetry between this and the SStx() method:
		SStt = 0;
	    for(int i = 0;  i < yData.length; i++){
			SStt += (i*dt - tMean)*(i*dt - tMean);
		}
	}
	
	// estimate() uses a first order least squares approximation to calculate the slope of the data:
	public double estimate(double y) {
		v0 = v;
		Push(yData, y);
		SSty = SStx(yData);
		v = SSty / SStt;
		a = (v - v0) / dt;
		return v;
	}
	
	// first() lets us query the 1st derivative without having to recompute it:
	public double first(){
		return v;
	}
	
	// second() lets us query the 2nd derivative:
	public double second(){
		return a;
	}
	
	private double SStx(double[] data){
		double mean  = Mean(data);
		double sum = 0.0;
	    for(int i=0; i<data.length; i++){
			sum += (i*dt - tMean)*(data[i] - mean);
		}
	    return sum;
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
	// all elements forward, and discards the last element:
	private void Push(double[] data, double x) {
		for (int i = data.length - 1; i > 0; i--) {
			data[i] = data[i - 1];
		}
		data[0] = x;
	}
	
	public void reset(){
		v0 = 0;
		v = 0;
		a = 0;
		for(int i = 0; i < yData.length; i++){
			yData[i] = 0.0;
		}
	}
	
}

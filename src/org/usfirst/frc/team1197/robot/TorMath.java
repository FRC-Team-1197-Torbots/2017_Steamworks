package org.usfirst.frc.team1197.robot;

public class TorMath {
	public static double sign(double x){
		if (x > 0.0)
			return 1.0;
		else if (x < 0.0)
			return -1.0;
		else
			return 0.0;
	}
}

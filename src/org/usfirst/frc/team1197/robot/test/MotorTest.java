package org.usfirst.frc.team1197.robot.test;

import org.usfirst.frc.team1197.robot.DriveHardware;

public class MotorTest extends Test {

	private DriveHardware hardware;
	
	private boolean leftMotorActuallyOnLeft;
	private boolean rightMotorActuallyOnRight;
	
	private boolean leftMotorDisconnected;
	private boolean rightMotorDisconnected;
	
	private boolean actualLeftOutputReversed;
	private boolean actualRightOutputReversed;

	public MotorTest(DriveHardware dh) {
		hardware = dh;
		setNumberOfSubtests(4);
	}

	@Override
	protected void setup() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void runTest() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void reset() {
		// TODO Auto-generated method stub

	}
	
	private void printResults() {
		// TODO Auto-generated method stub
		
	}

}

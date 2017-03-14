package org.usfirst.frc.team1197.robot.test;

public abstract class Test {
	
	protected int numberOfSubtests = 1;
	protected int numberOfSubtestsPassed = 0;

	public Test() {
	}
	
	public abstract Result run();
	
	protected Result testResult() {
		if (numberOfSubtestsPassed == numberOfSubtests) {
			return Result.PASS;
		}
		return Result.FAIL;
	}

}

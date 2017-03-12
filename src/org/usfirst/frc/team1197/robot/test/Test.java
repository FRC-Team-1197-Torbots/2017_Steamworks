package org.usfirst.frc.team1197.robot.test;

public abstract class Test {
	
	enum Result {PASS, FAIL};

	public Test(Object objectToTest) {
	}
	
	public abstract Result run();

}

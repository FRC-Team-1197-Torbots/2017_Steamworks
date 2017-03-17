package org.usfirst.frc.team1197.robot.test;

public abstract class Test {
	
	private int numberOfSubtests = 1;
	private int numberOfSubtestsPassed = 0;

	
	private static boolean enterButton = false;
	private static boolean resetButton = false;
	private static boolean enterButtonLast = false;
	private static boolean resetButtonLast = false;
	private static boolean enterButtonDownEvent = false;
	private static boolean resetButtonDownEvent = false;

	private boolean isFirstPass;
	private boolean isComplete;

	public Test() {
		isFirstPass = true;
	}
	
	protected void setNumberOfSubtests(int n){
		numberOfSubtests = n;
	}
	
	protected void incrementSubtestsPassed(){
		numberOfSubtestsPassed++;
	}
	
	protected abstract void setup();
	protected abstract void runTest();
	protected abstract void reset();
	
	public void run() {
		if (isFirstPass) {
			isFirstPass = false;
			isComplete = false;
			numberOfSubtestsPassed = 0;
			setup();
		}
		runTest();
		if (isComplete) {
			reset();
		}
	}
	
	public static void setButtons(boolean enter, boolean reset) {
		enterButtonLast = enterButton;
		resetButtonLast = resetButton;
		enterButton = enter;
		resetButton = reset;
		if (enterButton && !enterButtonLast) {
			enterButtonDownEvent = true;
			resetButtonDownEvent = false; // Can only have 1 at a time.
		}
		if (resetButton && !resetButtonLast) {
			resetButtonDownEvent = true;
			enterButtonDownEvent = false; // By checking the reset button 2nd, we give it priority.
		}
	}
	
	protected boolean enterButtonDownEvent(){
		if(enterButtonDownEvent) {
			enterButtonDownEvent = false; // Ensures the event is only processed once.
			return true;
		}
		return false;
	}
	
	protected boolean resetButtonDownEvent(){
		if(resetButtonDownEvent) {
			resetButtonDownEvent = false; // Ensures the event is only processed once.
			return true;
		}
		return false;
	}
	
	protected Result result() {
		if (numberOfSubtestsPassed == numberOfSubtests) {
			return Result.PASS;
		}
		return Result.FAIL;
	}
	
	protected boolean isComplete() {
		return isComplete;
	}
	
	protected void setComplete(boolean b) {
		isComplete = b;
	}
	

}

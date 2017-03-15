package org.usfirst.frc.team1197.robot.test;

public abstract class Test {
	
	private int numberOfSubtests = 1;
	private int numberOfSubtestsPassed = 0;

	
	protected boolean enterButton = false;
	protected boolean resetButton = false;
	protected boolean enterButtonLast = false;
	protected boolean resetButtonLast = false;

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
	protected abstract void run();
	protected abstract void reset();
	
	public void run(boolean enterButton, boolean resetButton) {
		enterButtonLast = this.enterButton;
		resetButtonLast = this.resetButton;
		this.enterButton = enterButton;
		this.resetButton = resetButton;
		if (isFirstPass) {
			isFirstPass = false;
			isComplete = false;
			numberOfSubtestsPassed = 0;
			setup();
		}
		run();
		if (isComplete) {
			reset();
		}
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

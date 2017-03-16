package org.usfirst.frc.team1197.robot.test;
import org.usfirst.frc.team1197.robot.DriveController;

import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorDrive;
import org.usfirst.frc.team1197.robot.TorTrajectory;

public class DriveControllerTest extends Test{
	
	private enum TestPhase {
		IDLE, BEGIN, FORWARDTRAJ, FORWARDSTALL, BACKWARDTRAJ, BACKWARDSTALL,
		RIGHTTRAJ, RIGHTSTALL, LEFTTRAJ, LEFTSTALL, COMPLETE
	}
	TestPhase testPhase = TestPhase.IDLE;
	
	private boolean isFirstPass = true;
	private boolean executeButton;
	private boolean executeButtonLast;
	
	private TorTrajectory forward;
	private TorTrajectory backward;
	private TorTrajectory rightTurn;
	private TorTrajectory leftTurn;
	private TorDrive drive;
	private DriveController controller;
	
	private boolean rightEncoderDirection;
	private boolean leftEncoderDirection;
	
	private boolean posWithinTolerance;
	private boolean velWithinTolerance;
	private boolean hedWithinTolerance;
	
	public DriveControllerTest(DriveController dc, TorDrive drive){
		setNumberOfSubtests(8);
		controller = dc;
		this.drive = drive;
		forward = new LinearTrajectory(1.0);
		backward = new LinearTrajectory(-1.0);
		rightTurn = new PivotTrajectory(90);
		leftTurn = new PivotTrajectory(-90);
	}
	
	protected void setup(){
		testPhase = TestPhase.BEGIN;
		controller.setMotionProfilingActive();
		posWithinTolerance = true;
		velWithinTolerance = true;
		hedWithinTolerance = true;
		reset();
	}

	public void run(boolean execute) {
		executeButtonLast = executeButton;
		executeButton = execute;
		run();
	}
	
	protected void run(){
		if (isFirstPass) {
			isFirstPass = false;
			setup();
		}
		testMotionProfile();
	}
	
	public void testMotionProfile(){
		switch(testPhase){
		case IDLE:
			break;
		case BEGIN:
			System.out.println("Ready to test forward trajectory.");
			System.out.println("> press A to execute the test.");
			testPhase = TestPhase.FORWARDTRAJ;
			break;
		case FORWARDTRAJ:
			if(executeButton && !executeButtonLast){
				testLinearTrajectory(forward);
				testPhase = TestPhase.FORWARDSTALL;
			}
			break;
		case FORWARDSTALL:
			if(forward.isComplete()){
				printEncoderError();
				printLinearToleranceError();
				reset();
				System.out.println();
				System.out.println("Ready to test backward trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.BACKWARDTRAJ;
			}
			break;
		case BACKWARDTRAJ:
			if(executeButton && !executeButtonLast){
				testLinearTrajectory(backward);
				testPhase = TestPhase.BACKWARDSTALL;
			}
			break;
		case BACKWARDSTALL:
			if(backward.isComplete()){
				printEncoderError();
				printLinearToleranceError();
				reset();
				System.out.println();
				System.out.println("Ready to test right trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.RIGHTTRAJ;
			}
			break;
		case RIGHTTRAJ:
			if(executeButton && !executeButtonLast){
				testPivotTrajectory(rightTurn);
				testPhase = TestPhase.RIGHTSTALL;
			}
			break;
		case RIGHTSTALL:
			if(rightTurn.isComplete()){
				printEncoderError();
				printPivotToleranceError();
				reset();
				System.out.println();
				System.out.println("Ready to test left trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.LEFTTRAJ;
			}
			break;
		case LEFTTRAJ:
			if(executeButton && !executeButtonLast){
				testPivotTrajectory(leftTurn);
				testPhase = TestPhase.LEFTSTALL;
			}
			break;
		case LEFTSTALL:
			if(leftTurn.isComplete()){
				printEncoderError();
				printPivotToleranceError();
				reset();
				System.out.println();
				testPhase = TestPhase.COMPLETE;
			}
			break;
		case COMPLETE:
			System.out.println("Completed DriveController test. Result: " + result());
			reset();
			controller.setMotionProfilingInactive();
			break;
		}
	}

	public void testLinearTrajectory(TorTrajectory traj){
		drive.executeTrajectory(traj);
		if(traj.getGoalPos() > 0){
			if(controller.hardware.getRightEncoder() > 0){
				rightEncoderDirection = true;
			}
			else{
				rightEncoderDirection = false;
			}
			if(controller.hardware.getLeftEncoder() > 0){
				leftEncoderDirection = true;
			}
			else{
				leftEncoderDirection = false;
			}
		}
		else{
			if(controller.hardware.getRightEncoder() < 0){
				rightEncoderDirection = true;
			}
			else{
				rightEncoderDirection = false;
			}
			if(controller.hardware.getLeftEncoder() < 0){
				leftEncoderDirection = true;
			}
			else{
				leftEncoderDirection = false;
			}
		}
		if(Math.abs(controller.getPositionError()) >= controller.getPositionTolerance()){
			posWithinTolerance = false;
		}
		if(Math.abs(controller.getVelocityError()) >= controller.getVelocityTolerance()){
			velWithinTolerance = false;
		}
	}

	public void testPivotTrajectory(TorTrajectory traj){
		drive.executeTrajectory(traj);
		if(traj.getGoalHeading() > 0){
			if(controller.hardware.getRightEncoder() < 0){
				rightEncoderDirection = true;
			}
			else{
				rightEncoderDirection = false;
			}
			if(controller.hardware.getLeftEncoder() > 0){
				leftEncoderDirection = true;
			}
			else{
				leftEncoderDirection = false;
			}
		}
		else{
			if(controller.hardware.getRightEncoder() > 0){
				rightEncoderDirection = true;
			}
			else{
				rightEncoderDirection = false;
			}
			if(controller.hardware.getLeftEncoder() < 0){
				leftEncoderDirection = true;
			}
			else{
				leftEncoderDirection = false;
			}
		}
		if(Math.abs(controller.getHeadingError()) >= controller.getHeadingTolerance()){
			hedWithinTolerance = false;
		}
	}
	
	public void printEncoderError(){
		if(rightEncoderDirection && !leftEncoderDirection){
			System.err.println("TEST FAILED (Encoder): Left encoder is not going the right direction.");
		}
		else if(!rightEncoderDirection && leftEncoderDirection){
			System.err.println("TEST FAILED (Encoder): Right encoder is not going the right direction.");
		}
		else if(!rightEncoderDirection && !leftEncoderDirection){
			System.err.println("TEST FAILED (Encoder): Both encoders are not going the right direction.");
		}
		else{
			System.out.println("TEST PASSED (Encoder): Both encoders are going the right direction.");
			incrementSubtestsPassed();
		}
	}
	
	public void printLinearToleranceError(){
		if(!posWithinTolerance && velWithinTolerance){
			System.err.println("TEST FAILED (Position): The position is not within the tolerance");
		}
		else if(posWithinTolerance && !velWithinTolerance){
			System.err.println("TEST FAILED (Velocity): The velocity is not within the tolernace");
		}
		else if(!posWithinTolerance && !velWithinTolerance){
			System.err.println("TEST FAILED (Pos&Vel): Both the position and the velocity is not within the tolernace");
		}
		else{
			System.out.println("TEST PASSED (Translation): The position and velocity is within the tolerance.");
			incrementSubtestsPassed();
		}
	}
	
	public void printPivotToleranceError(){
		if(!hedWithinTolerance){
			System.err.println("TEST FAILED (Heading): The heading is not within the tolerance");
		}
		else{
			System.out.println("TEST PASSED (Rotation): The heading is within the tolerance");
			incrementSubtestsPassed();
		}
	}
	
	protected void reset() {
		controller.hardware.resetEncoder();
		posWithinTolerance = true;
		velWithinTolerance = true;
		hedWithinTolerance = true;
	}
}

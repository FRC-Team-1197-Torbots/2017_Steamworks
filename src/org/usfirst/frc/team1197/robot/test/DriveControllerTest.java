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
	
	private TorTrajectory forward;
	private TorTrajectory backward;
	private TorTrajectory rightTurn;
	private TorTrajectory leftTurn;
	private TorDrive drive;
	private DriveController controller;
	
	private boolean rightEncoderDirection;
	private boolean leftEncoderDirection;
	
	private boolean gyroDirection;
	
	private boolean posWithinTolerance;
	private boolean velWithinTolerance;
	private boolean hedWithinTolerance;
	
	public DriveControllerTest(DriveController dc, TorDrive drive){
		setNumberOfSubtests(4);
		controller = dc;
		this.drive = drive;
		forward = new LinearTrajectory(1.0);
		backward = new LinearTrajectory(-2.0);
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

	protected void runTest(){
		switch(testPhase){
		case IDLE:
			break;
		case BEGIN:
			System.out.println("Ready to test forward trajectory.");
			System.out.println("> press A to execute the test.");
			testPhase = TestPhase.FORWARDTRAJ;
			break;
		case FORWARDTRAJ:
			if(enterButtonDownEvent()){
				System.out.println("Active (Before): " + drive.controller.activeTrajectory);
				System.out.println("Next (Before): " + drive.controller.nextTrajectory);
				drive.executeTrajectory(forward);
				System.out.println("Active (After): " + drive.controller.activeTrajectory);
				System.out.println("Next (After): " + drive.controller.nextTrajectory);
				testPhase = TestPhase.FORWARDSTALL;
			}
			break;
		case FORWARDSTALL:
			testLinearTrajectory(forward);
			if(forward.isComplete()){
				printEncoderError();
				printLinearToleranceError();
				drive.controller.shiftToLowGear();
				drive.controller.shiftToHighGear();
				System.out.println();
				System.out.println("Ready to test backward trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.BACKWARDTRAJ;
			}
			break;
		case BACKWARDTRAJ:
			if(enterButtonDownEvent()){
				drive.executeTrajectory(backward);
				testPhase = TestPhase.BACKWARDSTALL;
			}
			break;
		case BACKWARDSTALL:
			testLinearTrajectory(backward);
			if(backward.isComplete()){
				printEncoderError();
				printLinearToleranceError();
				System.out.println();
//				System.out.println("For testing the pivot trajectories, place the robot on the ground.");
//				System.out.println("Ready to test right trajectory.");
//				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.COMPLETE;
			}
			break;
		case RIGHTTRAJ:
			if(enterButtonDownEvent()){
				testPivotTrajectory(rightTurn);
				testPhase = TestPhase.RIGHTSTALL;
			}
			break;
		case RIGHTSTALL:
			if(rightTurn.isComplete()){
				printGyroError();
				printPivotToleranceError();
				System.out.println();
				System.out.println("Ready to test left trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.LEFTTRAJ;
			}
			break;
		case LEFTTRAJ:
			if(enterButtonDownEvent()){
				testPivotTrajectory(leftTurn);
				testPhase = TestPhase.LEFTSTALL;
			}
			break;
		case LEFTSTALL:
			if(leftTurn.isComplete()){
				printGyroError();
				printPivotToleranceError();
				System.out.println();
				testPhase = TestPhase.COMPLETE;
			}
			break;
		case COMPLETE:
			if(rightEncoderDirection && leftEncoderDirection){
				incrementSubtestsPassed();
			}
			if(posWithinTolerance && velWithinTolerance){
				incrementSubtestsPassed();
			}
			if(gyroDirection){
				incrementSubtestsPassed();
			}
			System.out.println("Completed DriveController test. Result: " + result());
			reset();
			controller.setMotionProfilingInactive();
			testPhase = TestPhase.IDLE;
			break;
		}
	}

	public void testLinearTrajectory(TorTrajectory traj){
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
			if(controller.hardware.getHeading() > 0){
				gyroDirection = true;
			}
			else{
				gyroDirection = false;
			}
		}
		else{
			if(controller.hardware.getHeading() < 0){
				gyroDirection = true;
			}
			else{
				gyroDirection = false;
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
		}
	}
	
	public void printGyroError(){
		if(gyroDirection){
			System.out.println("TEST PASSED (Gyro): The gyro is going the right direction.");
		}
		else{
			System.err.println("TEST FAILED (Gyro): The gyro is not going the right direction.");
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
		}
	}
	
	public void printPivotToleranceError(){
		if(!hedWithinTolerance){
			System.err.println("TEST FAILED (Heading): The heading is not within the tolerance");
		}
		else{
			System.out.println("TEST PASSED (Rotation): The heading is within the tolerance");
		}
	}
	
	public boolean testTrajComplete(TorTrajectory traj){
		return Math.abs(drive.controller.hardware.getPosition()) > Math.abs(traj.getGoalPos()*0.5) 
				&& drive.controller.hardware.getVelocity() == 0;
	}
	
	protected void reset() {
		controller.hardware.resetEncoder();
		controller.hardware.resetGyro();
		posWithinTolerance = true;
		velWithinTolerance = true;
		hedWithinTolerance = true;
	}
}

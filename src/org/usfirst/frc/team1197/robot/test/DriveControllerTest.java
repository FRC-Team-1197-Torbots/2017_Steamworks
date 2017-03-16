package org.usfirst.frc.team1197.robot.test;
import org.usfirst.frc.team1197.robot.DriveController;

import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorDrive;
import org.usfirst.frc.team1197.robot.TorTrajectory;

public class DriveControllerTest extends Test{
	
	private enum TestPhase {
		IDLE, BEGIN, FORWARDTRAJ, FORWARDSTALL, BACKWARDTRAJ, RIGHTTRAJ, LEFTTRAJ, COMPLETE
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
	
	public DriveControllerTest(DriveController dc){
		controller = dc;
		drive = new TorDrive();
		forward = new LinearTrajectory(1.0);
		backward = new LinearTrajectory(-1.0);
		rightTurn = new PivotTrajectory(90);
		leftTurn = new PivotTrajectory(-90);
	}
	
	protected void setup(){
		testPhase = TestPhase.FORWARDTRAJ;
		posWithinTolerance = true;
		velWithinTolerance = true;
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
			controller.hardware.resetEncoder();
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
				if(rightEncoderDirection && !leftEncoderDirection){
					System.err.println("TEST FAILED (Forward: Encoder): Left encoder is not going the right direction.");
				}
				else if(!rightEncoderDirection && leftEncoderDirection){
					System.err.println("TEST FAILED (Forward: Encoder): Right encoder is not going the right direction.");
				}
				else if(!rightEncoderDirection && !leftEncoderDirection){
					System.err.println("TEST FAILED (Forward: Encoder): Both encoders are not going the right direction.");
				}
				else{
					System.out.println("TEST PASSED (Forward: Encoder): Both encoders are going the right direction.");
				}
				if(!posWithinTolerance && velWithinTolerance){
					System.err.println("TEST FAILED (Forward: Position): The position is not within the tolerance");
				}
				else if(posWithinTolerance && !velWithinTolerance){
					System.err.println("TEST FAILED (Forward: Velocity): The velocity is not within the tolernace");
				}
				else if(!posWithinTolerance && !velWithinTolerance){
					System.err.println("TEST FAILED (Forward: Pos&Vel): Both the position and the velocity is not within the tolernace");
				}
				else{
					System.out.println("TEST PASSED (Forward: Translation): The position and velocity is within the tolerance.");
				}
				System.out.println("Ready to test backward trajectory.");
				System.out.println("> press A to execute the test.");
				testPhase = TestPhase.IDLE;
			}
			break;
		case BACKWARDTRAJ:
			if(executeButton && !executeButtonLast){
				
			}
			break;
		case RIGHTTRAJ:
			break;
		case LEFTTRAJ:
			break;
		case COMPLETE:
			break;
		}
	}
	
	public void testLinearTrajectory(TorTrajectory traj){
		//Execute trajectory
		drive.executeTrajectory(traj);
		//Check if the encoders are going the right direction
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
		//Compare the difference between target and current and see if its within the tolerance
		if(Math.abs(controller.getPositionError()) >= controller.getPositionTolerance()){
			posWithinTolerance = false;
		}
		if(Math.abs(controller.getVelocityError()) >= controller.getVelocityTolerance()){
			velWithinTolerance = false;
		}
	}
	
	public void testPivotTrajectory(TorTrajectory traj){
		//Execute trajectory
				drive.executeTrajectory(traj);
				//Check if the encoders are going the right direction
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
				//Compare the difference between target and current and see if its within the tolerance
				if(Math.abs(controller.getHeadingError()) >= controller.getHeadingTolerance()){
					hedWithinTolerance = false;
				}
	}

	@Override
	protected void reset() {
		// TODO Auto-generated method stub
		
	}
}

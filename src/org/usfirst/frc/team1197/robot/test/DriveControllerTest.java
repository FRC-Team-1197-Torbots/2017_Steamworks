package org.usfirst.frc.team1197.robot.test;
import org.usfirst.frc.team1197.robot.DriveController;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorDrive;
import org.usfirst.frc.team1197.robot.TorTrajectory;

public class DriveControllerTest extends Test{
	
	private TorTrajectory forward;
	private TorTrajectory backward;
	private TorTrajectory rightTurn;
	private TorTrajectory leftTurn;
	private TorDrive drive;
	private DriveController controller;
	
	private boolean rightEncoderDirection;
	private boolean leftEncoderDirection;
	
	public DriveControllerTest(DriveController dc){
		controller = dc;
		drive = new TorDrive();
		forward = new LinearTrajectory(1.0);
		backward = new LinearTrajectory(-1.0);
		rightTurn = new PivotTrajectory(90);
		leftTurn = new PivotTrajectory(-90);
		numberOfSubtests = 4;
	}

	public void run() {
		testLinearTrajectory(forward);
		//don't run the next testTrajectory until the previous one is over
		testLinearTrajectory(backward);
		//don't run the next testTrajectory until the previous one is over
		testPivotTrajectory(rightTurn);
		//don't run the next testTrajectory until the previous one is over
		testPivotTrajectory(leftTurn);
	}
	
	public Result testLinearTrajectory(TorTrajectory traj){
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
//		if(controller.hardware.)
		   //for position, heading, velocity
		return null;
	}
	
	public Result testPivotTrajectory(TorTrajectory traj){
		return null;
	}
}

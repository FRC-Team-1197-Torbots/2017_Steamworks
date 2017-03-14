package org.usfirst.frc.team1197.robot.test;
import org.usfirst.frc.team1197.robot.LinearTrajectory;
import org.usfirst.frc.team1197.robot.PivotTrajectory;
import org.usfirst.frc.team1197.robot.TorDrive;
import org.usfirst.frc.team1197.robot.TorTrajectory;

public class TorMotionProfileTest extends Test{
	
	TorTrajectory forward;
	TorTrajectory backward;
	TorTrajectory rightTurn;
	TorTrajectory leftTurn;
	TorDrive drive;
	
	public TorMotionProfileTest(){
		drive = new TorDrive();
		forward = new LinearTrajectory(1.0);
		backward = new LinearTrajectory(-1.0);
		rightTurn = new PivotTrajectory(90);
		leftTurn = new PivotTrajectory(-90);
	}

	public Result run() {
		testTrajectory(forward);
		//don't run the next testTrajectory until the previous one is over
		testTrajectory(backward);
		//don't run the next testTrajectory until the previous one is over
		testTrajectory(rightTurn);
		//don't run the next testTrajectory until the previous one is over
		testTrajectory(leftTurn);
		return null;
	}
	
	public void testTrajectory(TorTrajectory traj){
		//Execute trajectory
		//Check if the encoders are going the right direction
		//Compare the difference between target and current and see if its within the tolerance
		   //for position, heading, velocity
		//Maybe make a list of strings to store an error message and print out at the very end?
	}
}

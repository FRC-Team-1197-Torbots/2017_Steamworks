package org.usfirst.frc.team1197.robot.test;
import org.usfirst.frc.team1197.robot.DriveHardware;

public class DriveHardwareTest extends Test {
	
	private DriveHardware hardware;

	public DriveHardwareTest(DriveHardware dh) {
		hardware = dh;
		numberOfSubtests = 4;
	}

	@Override
	public Result run() {
		// Prompt user to manually roll wheels forward
		
		// 1) Test to make sure all talons are on the correct side
		
		// 2) Test to make sure followers follow the master
		
		// 3) Test to make sure output goes in right direction
		
		// 4) Check to make sure encoder reading has correct sign

		return testResult();
	}

}

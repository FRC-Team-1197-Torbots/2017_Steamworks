package org.usfirst.frc.team1197.robot.test;

import org.usfirst.frc.team1197.robot.DriveHardware;

public class DriveHardwareTest extends Test {
	
	private DriveHardware hardware;
	private EncoderTest encoderTest;
	private MotorTest motorTest;

	private enum TestPhase {
		ENCODER_TEST, MOTOR_TEST
	}
	TestPhase testPhase;

	public DriveHardwareTest(DriveHardware dh) {
		hardware = dh;
		setNumberOfSubtests(2);
		encoderTest = new EncoderTest(hardware);
		motorTest = new MotorTest(hardware);
	}

	protected void setup() {
		encoderTest.setup();
		motorTest.setup();
		testPhase = TestPhase.ENCODER_TEST;
	}
	
	protected void runTest() {
		switch(testPhase){
		case ENCODER_TEST:
			// Test that encoder direction matches physical direction, and that
			// both encoders are hooked up to the correct sides of the robot.
			encoderTest.run();
			if (encoderTest.isComplete()) {
				testPhase = TestPhase.MOTOR_TEST;
			}
			break;
		case MOTOR_TEST:
			// Test to make sure all motors on the same gearbox have consistent
			// polarity with each other, that follower motors follow the master,
			// that the motors are on the correct sides, and that the motors have
			// correct polarity.
			break;
		default:
			break;
		}
	}

	@Override
	protected void reset() {
		encoderTest.reset();
		motorTest.reset();
	}
	

	
}
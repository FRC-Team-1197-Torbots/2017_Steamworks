package org.usfirst.frc.team1197.robot.test;

import org.usfirst.frc.team1197.robot.DriveHardware;

class EncoderTest extends Test {
	
	private DriveHardware hardware;

	private enum TestPhase {
		BEGIN, CHECKING_LEFT, CHECKING_RIGHT, COMPLETE
	}
	private TestPhase testPhase;

	private boolean leftEncoderActuallyOnLeft;
	private boolean rightEncoderActuallyOnRight;

	private boolean leftEncoderOnBoth = false;
	private boolean rightEncoderOnBoth = false;
	
	private boolean actualLeftSensorReversed;
	private boolean actualRightSensorReversed;

	private boolean leftEncoderDisconnected;
	private boolean rightEncoderDisconnected;
	
	public EncoderTest(DriveHardware dh) {
		hardware = dh;
		setNumberOfSubtests(3);
	}
	
	@Override
	protected void setup() {
		hardware.choosePercentVbus();
		hardware.resetEncoder();
		testPhase = TestPhase.BEGIN;
	}

	@Override
	protected void run() {
		switch (testPhase) {
		case BEGIN:
			hardware.resetEncoder();
			System.out.println("Manually roll the LEFT wheels FORWARD.");
			System.out.println("> When finished, press A to continue or press B to start over for this side.\n");
			testPhase = TestPhase.CHECKING_LEFT;
			break;
		case CHECKING_LEFT:
			if (resetButton && !resetButtonLast) {
				hardware.resetEncoder();
				System.out.println("Encoder reset. You can now try again.");
			} else if (enterButton && !enterButtonLast) {
				if (Math.abs(hardware.getLeftEncoder()) > Math.abs(hardware.getRightEncoder())) {
					leftEncoderActuallyOnLeft = true;
					if (hardware.getLeftEncoder() > 0) {
						actualLeftSensorReversed = false;
						leftEncoderDisconnected = false;
					} else if (hardware.getLeftEncoder() < 0) {
						actualLeftSensorReversed = true;
						leftEncoderDisconnected = false;
					} else {
						leftEncoderDisconnected = true;
					}
				} else if (Math.abs(hardware.getLeftEncoder()) < Math.abs(hardware.getRightEncoder())) {
					rightEncoderActuallyOnRight = false;
					if (hardware.getRightEncoder() > 0) {
						actualLeftSensorReversed = false;
						leftEncoderDisconnected = false;
					} else if (hardware.getRightEncoder() < 0) {
						actualLeftSensorReversed = true;
						leftEncoderDisconnected = false;
					} else {
						leftEncoderDisconnected = true;
					}
				}
				hardware.resetEncoder();
				System.out.println("Manually roll the RIGHT wheels FORWARD.");
				System.out.println("> When finished, press A to continue or press B to start over for this side.\n");
				testPhase = TestPhase.CHECKING_RIGHT;
			}
			break;
		case CHECKING_RIGHT:
			if (resetButton && !resetButtonLast) {
				hardware.resetEncoder();
				System.out.println("Encoder reset. You can now try again.");
			} else if (enterButton && !enterButtonLast) {
				if (Math.abs(hardware.getLeftEncoder()) < Math.abs(hardware.getRightEncoder())) {
					if (!rightEncoderActuallyOnRight) {
						rightEncoderOnBoth = true;
					}
					rightEncoderActuallyOnRight = true;
					if (hardware.getRightEncoder() > 0) {
						actualRightSensorReversed = false;
						rightEncoderDisconnected = false;
					} else if (hardware.getRightEncoder() < 0) {
						actualRightSensorReversed = true;
						rightEncoderDisconnected = false;
					} else {
						rightEncoderDisconnected = true;
					}
				} else if (Math.abs(hardware.getLeftEncoder()) > Math.abs(hardware.getRightEncoder())) {
					if (leftEncoderActuallyOnLeft) {
						leftEncoderOnBoth = true;
					}
					leftEncoderActuallyOnLeft = false;
					if (hardware.getLeftEncoder() > 0) {
						actualRightSensorReversed = false;
						rightEncoderDisconnected = false;
					} else if (hardware.getLeftEncoder() < 0) {
						actualRightSensorReversed = true;
						rightEncoderDisconnected = false;
					} else {
						rightEncoderDisconnected = true;
					}
				}
			}
			printResults();
			testPhase = TestPhase.COMPLETE;
			setComplete(true);
			break;
		case COMPLETE:
			break;
		default:
			break;
		}
	}

	@Override
	protected void reset() {
		// TODO Auto-generated method stub
		
	}
	
	private void printResults() {
		// Test 1: Are both encoders are connected?
		if (leftEncoderDisconnected && rightEncoderDisconnected) {
			System.err.println("TEST FAILED (encoder connection): Both encoders are disconnected.");
			System.err.println("TEST FAILED (encoder sides): Both encoders are disconnected.");
			System.err.println("TEST FAILED (encoder direction): Both encoders are disconnected.");
		} else if (leftEncoderDisconnected && !rightEncoderDisconnected) {
			System.err.println("TEST FAILED (encoder connection): Left encoder is disconnected.");
			System.err.println("TEST FAILED (encoder sides): Left encoder is disconnected.");
			System.err.println("TEST FAILED (encoder direction): Left encoder is disconnected.");
		} else if (!leftEncoderDisconnected && rightEncoderDisconnected) {
			System.err.println("TEST FAILED (encoder connection): Right encoder is disconnected.");
			System.err.println("TEST FAILED (encoder sides): Right encoder is disconnected.");
			System.err.println("TEST FAILED (encoder direction): Right encoder is disconnected.");
		} else {
			System.out.println("TEST PASSED (encoder connection): Both encoders are connected.");
			incrementSubtestsPassed();
			// Test 2: Are both encoders on the correct sides?
			if (leftEncoderActuallyOnLeft && rightEncoderActuallyOnRight) {
				System.out.println(
						"TEST PASSED (encoder sides): Both encoders' software labels match their actual sides.");
				incrementSubtestsPassed();
			} else if (leftEncoderOnBoth || rightEncoderOnBoth) {
				System.err
						.println("TEST FAILED (encoder sides): One or both encoders are hooked up to both sides.");
			} else if (!leftEncoderActuallyOnLeft && !rightEncoderActuallyOnRight) {
				System.err.println("TEST FAILED (encoder sides): Left and right encoders are swapped.");
			} else if (leftEncoderActuallyOnLeft && !rightEncoderActuallyOnRight) {
				System.err.println("TEST FAILED (encoder sides): Both encoders are actualy on the left.");
			} else if (!leftEncoderActuallyOnLeft && rightEncoderActuallyOnRight) {
				System.err.println("TEST FAILED (encoder sides): Both encoders are actualy on the right.");
			}
			// Test 3: Do both encoders know which way "forward" is?
			if (!actualLeftSensorReversed && !actualRightSensorReversed) {
				System.out.println("TEST PASSED (encoder direction): Both encoders are oriented correctly");
				incrementSubtestsPassed();
			} else if (!actualLeftSensorReversed && actualRightSensorReversed) {
				System.err.println("TEST FAILED (encoder direction): Right encoder reversed.");
			} else if (actualLeftSensorReversed && !actualRightSensorReversed) {
				System.err.println("TEST FAILED (encoder direction): Left encoder reversed.");
			} else {
				System.err.println("TEST FAILED (encoder direction): Both encoders reversed.");
			}
		}
	}

}

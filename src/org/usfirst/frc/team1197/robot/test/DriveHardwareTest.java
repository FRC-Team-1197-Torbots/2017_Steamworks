package org.usfirst.frc.team1197.robot.test;

import org.usfirst.frc.team1197.robot.DriveHardware;

import com.ctre.CANTalon;

public class DriveHardwareTest extends Test {
	
	private DriveHardware hardware;

	private enum TestPhase {
		CALIBRATION, CHECKING_SIDES, CHECKING_FOLLOWERS, CHECKING_SENSOR_DIRECTION, CHECKING_MOTOR_DIRECTION
	}
	TestPhase testPhase;

	private enum CalibrationState {
		BEGIN, CHECKING_LEFT, CHECKING_RIGHT, COMPLETE, IDLE
	}
	CalibrationState calibrationState;

	private boolean isFirstPass = true;
	private boolean enterButton = false;
	private boolean resetButton = false;
	private boolean enterButtonLast = false;
	private boolean resetButtonLast = false;

	private boolean leftEncoderActuallyOnLeft;
	private boolean rightEncoderActuallyOnRight;
	private boolean leftMotorActuallyOnLeft;
	private boolean rightMotorActuallyOnRight;
	
	private boolean leftEncoderOnBoth = false;
	private boolean rightEncoderOnBoth = false;
	
	private boolean leftEncoderDisconnected;
	private boolean rightEncoderDisconnected;
	private boolean leftMotorDisconnected;
	private boolean rightMotorDisconnected;
	
	private boolean actualLeftSensorReversed;
	private boolean actualRightSensorReversed;
	private boolean actualLeftOutputReversed;
	private boolean actualRightOutputReversed;

	public DriveHardwareTest(DriveHardware dh) {
		hardware = dh;
		numberOfSubtests = 3;
	}

	private void setup() {
		numberOfSubtestsPassed = 0;
		hardware.choosePercentVbus();
		hardware.resetEncoder();
		testPhase = TestPhase.CALIBRATION;
		calibrationState = CalibrationState.BEGIN;
	}

	protected void run() {
		if (isFirstPass) {
			isFirstPass = false;
			setup();
		}
		// Test that encoder direction matches physical direction, 
		// and that both encoders are hooked up to the correct sides of the robot:
		calibrateEncoders();

		// 2) Test to make sure followers follow the master

		// 3) Test to make sure output goes in right direction

		// 4) Check to make sure encoder reading has correct sign
	}

	public void run(boolean enter, boolean reset) {
		enterButtonLast = enterButton;
		resetButtonLast = resetButton;
		enterButton = enter;
		resetButton = reset;
		run();
	}

	private void calibrateEncoders() {
		switch (calibrationState) {
		case BEGIN:
			hardware.resetEncoder();
			System.out.println("Manually roll the LEFT wheels FORWARD.");
			System.out.println("> When finished, press A to continue or press B to start over for this side.\n");
			calibrationState = CalibrationState.CHECKING_LEFT;
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
				calibrationState = CalibrationState.CHECKING_RIGHT;
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
						calibrationState = CalibrationState.IDLE;
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
						calibrationState = CalibrationState.IDLE;
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
				hardware.resetEncoder();
				calibrationState = CalibrationState.COMPLETE;
			}
			break;
		case COMPLETE:
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
				numberOfSubtestsPassed++;
				// Test 2: Are both encoders on the correct sides?
				if (leftEncoderActuallyOnLeft && rightEncoderActuallyOnRight) {
					System.out.println(
							"TEST PASSED (encoder sides): Both encoders' software labels match their actual sides.");
					numberOfSubtestsPassed++;
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
					numberOfSubtestsPassed++;
				} else if (!actualLeftSensorReversed && actualRightSensorReversed) {
					System.err.println("TEST FAILED (encoder direction): Right encoder reversed.");
				} else if (actualLeftSensorReversed && !actualRightSensorReversed) {
					System.err.println("TEST FAILED (encoder direction): Left encoder reversed.");
				} else {
					System.err.println("TEST FAILED (encoder direction): Both encoders reversed.");
				}
			}
			calibrationState = calibrationState.IDLE;
			break;
		case IDLE:
			break;
		default:
			break;
		}
	}
}
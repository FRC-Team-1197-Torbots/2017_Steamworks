package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1197.robot.test.DriveControllerTest;
import org.usfirst.frc.team1197.robot.test.DriveHardwareTest;
import org.usfirst.frc.team1197.robot.test.Test;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private CANTalon climbTalon;
	private CANTalon gearIntakeTalon;
	
	private Compressor compressor;

	private Solenoid gearPiston;
	private Solenoid gearIntakePiston;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
	private DigitalInput gearSwitch;
	private DigitalInput gearIntakeDetector;
	
	private DriveHardwareTest hardwareTest;
	private DriveControllerTest controllerTest;
	
	private TorDrive drive;
	private TorClimb climb;
	private TorGearIntake gearIntake;
	private TorGear gear;
	private TorAuto auto;
	protected static RobotMode mode;
	
    public Robot() {
//    	CameraServer server = CameraServer.getInstance();
//    	server.startAutomaticCapture("cam0", 0);
//    	server.putVideo("Gear Intake", 640, 480);
    	
    	mode = RobotMode.DISABLED;
    	
    	climbTalon = new CANTalon(10); 
    	gearIntakeTalon = new CANTalon(6);
    	
    	compressor = new Compressor();
    	
    	gearPiston = new Solenoid(2);
    	gearIntakePiston = new Solenoid(1);
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
    	
    	gearSwitch = new DigitalInput(0);
    	gearIntakeDetector = new DigitalInput(1);
    	
    	drive = new TorDrive(player1, autoBox);
    	climb = new TorClimb(climbTalon, player2);
    	gearIntake = new TorGearIntake(player2, gearIntakeTalon, gearIntakePiston, gearIntakeDetector);
    	gear = new TorGear(gearPiston, gearSwitch, player1);
    	auto = new TorAuto(drive, autoBox, gear);
    	
    	hardwareTest = new DriveHardwareTest(drive.controller.hardware);
    	controllerTest = new DriveControllerTest(drive.controller, drive);
    }

    public void autonomous() {
    	mode = RobotMode.AUTO;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
    	TorTrajectory.setRotationMirrored(!isRed());
    	auto.initialize();
    	auto.run();
    }

    public void operatorControl() {
    	mode = RobotMode.TELEOP;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
//    		if(player2.getRawButton(8)){
//    			gear.playerControl();
//    		}
//    		else{
//    			gear.autoControl();
//    		}
    		gearIntake.autoControl();
//    		climb.playerControl();
    	}
    }

    public void test() {
    	mode = RobotMode.TEST;
    	drive.controller.setClosedLoopConstants(mode);
    	drive.enable();
		while(isEnabled()){
//			Test.setButtons(getButtonA(), getButtonB());
//			controllerTest.run();
//			hardwareTest.run(getButtonA(), getButtonB());
//			compressor.start();
		}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		mode = RobotMode.DISABLED;
		drive.disable();
	}

	// Getting the left analog stick X-axis value from the xbox controller. 
	public double getLeftX(){
		return player1.getRawAxis(0);
	}

	// Getting the left analog stick Y-axis value from the xbox controller. 
	public double getLeftY(){
		return player1.getRawAxis(1);
	}

	// Getting the right analog stick X-axis value from the xbox controller. 
	public double getRightX(){
		return player1.getRawAxis(4);
	}

	// Getting the right trigger value from the xbox controller.
	public double getRightTrigger(){
		return player1.getRawAxis(3);
	}

	// Getting the left bumper button value from the xbox controller. 
	public boolean getShiftButton(){
		return player1.getRawButton(5);
	}

	public boolean getRightBumper(){
		return player1.getRawButton(6);
	}

	public boolean getButtonA(){
		return player1.getRawButton(1);
	}

	public boolean getButtonB(){
		return player1.getRawButton(2);
	}

	public boolean getButtonX(){
		return player1.getRawButton(3);
	}

	public boolean getButtonY(){
		return player1.getRawButton(4);
	}
	
	public boolean isRed(){
		return autoBox.getRawButton(4);
	}

}

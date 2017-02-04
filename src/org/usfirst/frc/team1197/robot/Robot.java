package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;


import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private CANTalon climbTalon;
	private CANTalon elevatorTalon;
	private CANTalon dumperTalon;
	private Compressor compressor;
	private Solenoid shift;
	private Joystick player1;
	private Joystick player2;
	private DigitalInput gearSwitch;
	private DigitalInput climbSwitch;
	private Solenoid gearPiston;
	private TorDrive drive;
	private TorGear gear;
	private TorClimb climb;
	private TorIntake intake;
	
    public Robot() {
    	climbTalon = new CANTalon(7);
    	elevatorTalon = new CANTalon(8);
    	dumperTalon = new CANTalon(9);
    	compressor = new Compressor();
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	climbSwitch = new DigitalInput(0);
    	gearSwitch = new DigitalInput(1);
    	gearPiston = new Solenoid(0);
    	shift = new Solenoid(0);
    	drive = new TorDrive(player1, shift);
    	gear = new TorGear(gearPiston, gearSwitch, player1);
    	climb = new TorClimb(climbTalon, climbSwitch, player2);
    	intake = new TorIntake(elevatorTalon, dumperTalon, player2);
    	
    }
    
    public void robotInit() {

    }

    public void autonomous() {
    	
    }

    public void operatorControl() {
    	drive.shiftToHighGear();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
    		climb.update();
    		gear.update();
    		intake.update();
    	}
    }

    public void test() {
		while(isEnabled()){
			compressor.start();
		}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		drive.shiftToLowGear();
		shift.set(false); 
		TorCAN.INSTANCE.SetDrive(0.0, 0.0);
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
}

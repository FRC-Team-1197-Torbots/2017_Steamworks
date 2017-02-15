package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private SerialPort port;
	
	private CANTalon climbTalon;
	private CANTalon elevatorTalon1;
	private CANTalon elevatorTalon2;
	private CANTalon dumperTalon;
	
	private Compressor compressor;
	
	private Solenoid shift;
	private Solenoid gearPiston;
	
	private Joystick player1;
	private Joystick player2;
	private Joystick autoBox;
	
	private DigitalInput climbSwitch;
	private DigitalInput gearSwitch;
	
	private TorDrive drive;
	private TorClimb climb;
	private TorIntake intake;
	private TorGear gear;
	private TorAuto auto;
	private TorLidar lidar;
	
    public Robot() {
    	port = new SerialPort(9600, SerialPort.Port.kOnboard);
    	
    	climbTalon = new CANTalon(7);
    	dumperTalon = new CANTalon(8);
    	elevatorTalon1 = new CANTalon(9);
    	elevatorTalon2 = new CANTalon(10);
    	
    	compressor = new Compressor();
    	
    	shift = new Solenoid(0);
    	gearPiston = new Solenoid(1);
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
    	
    	climbSwitch = new DigitalInput(1);
    	gearSwitch = new DigitalInput(0);
    	
    	lidar = new TorLidar(port);
    	drive = new TorDrive(player1, shift, autoBox);
    	climb = new TorClimb(climbTalon, climbSwitch, player2, lidar);
    	intake = new TorIntake(elevatorTalon1, elevatorTalon2, dumperTalon, player2);
    	gear = new TorGear(gearPiston, gearSwitch, player1);
    	auto = new TorAuto(intake, autoBox);
    }

    public void autonomous() {
    	drive.shiftToHighGearMotion();
    	auto.initialize();
    	auto.run();
    }

    public void operatorControl() {
    	drive.shiftToHighGearMotion();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
//    		climb.update();
    		intake.update();
    		gear.update();
    	}
    }

    public void test() {
		while(isEnabled()){
			compressor.start();
			if(player1.getRawButton(1)){
				climbTalon.set(-0.8);
			}
			else{
				climbTalon.set(0.0);
			}
		}
	}

	//Low-gear software wise, High-gear mechanically
	public void disabled() {
		climb.resetClimb();
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

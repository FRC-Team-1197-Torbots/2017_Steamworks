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
	
	private DigitalInput gearSwitch;
	
	private TorDrive drive;
	private TorClimb climb;
	private TorIntake intake;
	private TorGear gear;
	private TorAuto auto;
	protected static RobotMode mode;
	
    public Robot() {
    	mode = RobotMode.DISABLED;
    	
    	climbTalon = new CANTalon(10); 
    	dumperTalon = new CANTalon(4); 
    	elevatorTalon1 = new CANTalon(5); 
    	elevatorTalon2 = new CANTalon(6); 
    	
    	compressor = new Compressor();
    	
    	gearPiston = new Solenoid(1);
    	
    	player1 = new Joystick(0);
    	player2 = new Joystick(1);
    	autoBox = new Joystick(2);
    	
    	gearSwitch = new DigitalInput(0);
    	
    	drive = new TorDrive(player1, shift, autoBox);
    	climb = new TorClimb(climbTalon, player2);
    	intake = new TorIntake(elevatorTalon1, elevatorTalon2, dumperTalon, player2);
    	gear = new TorGear(gearPiston, gearSwitch, player1);
    	auto = new TorAuto(drive, intake, autoBox, gear);
    }

    public void autonomous() {
    	mode = RobotMode.AUTO;
    	drive.enable();
//    	drive.shiftToHighGear();
//    	TorTrajectory.setRotationMirrored(!isRed());
//    	auto.initialize();
//    	auto.run();
    }

    public void operatorControl() {
    	mode = RobotMode.TELEOP;
    	drive.enable();
    	while(isEnabled()){
    		drive.driving(getLeftY(), getLeftX(), getRightX(), getShiftButton(), getRightBumper(), 
					getButtonA(), getButtonB(), getButtonX(), getButtonY());
    		intake.update();
    		climb.manualClimb();
    		gear.Gear();
    	}
    }

    public void test() {
    	mode = RobotMode.TELEOP;
    	drive.enable();
		while(isEnabled()){
			compressor.start();
			if(player2.getRawButton(1)){
				elevatorTalon1.set(1.0);
			}
			else if(player2.getRawButton(2)){
				elevatorTalon2.set(1.0);
			}
			else if(player2.getRawButton(3)){
				dumperTalon.set(1.0);
			}
			else if(player2.getRawButton(5)){
				climbTalon.set(-1.0);
			}
			else{
				elevatorTalon1.set(0.0);
				elevatorTalon2.set(0.0);
				dumperTalon.set(0.0);
				climbTalon.set(0.0);
			}
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

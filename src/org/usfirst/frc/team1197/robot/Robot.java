package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends SampleRobot {
	private Compressor compressor;
	private CANTalon rightTalon1;
	private CANTalon rightTalon2;
	private CANTalon leftTalon1;
	private CANTalon leftTalon2;
	private Solenoid solenoid;
	private Joystick stick;
	private RobotDrive robot;

    public Robot() {
    	compressor = new Compressor();
    	rightTalon1 = new CANTalon(2); //right master
    	rightTalon2 = new CANTalon(3);
    	leftTalon1 = new CANTalon(4); //left master
    	leftTalon2 = new CANTalon(6);
    	stick = new Joystick(0);
    	solenoid = new Solenoid(1);
    	robot = new RobotDrive(leftTalon1, leftTalon2, rightTalon1, rightTalon2);
    }
    
    public void robotInit() {

    }

    public void autonomous() {

    }

    public void operatorControl() {
    	while(isEnabled()){
    		robot.arcadeDrive(stick);
    		if(stick.getRawButton(1)){
    			solenoid.set(true);
    		}
    		if(stick.getRawButton(2)){
    			solenoid.set(false);
    		}
    	}
    }

    public void test() {
    	while(isEnabled()){
    		compressor.start();
    	}
    }
}

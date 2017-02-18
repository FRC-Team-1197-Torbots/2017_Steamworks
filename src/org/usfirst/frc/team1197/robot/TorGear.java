package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class TorGear {

	private Solenoid gearPiston;
	private DigitalInput gearSwitch;
	private Joystick stick;
	private GearBackTraj gearBack;
	private TorDrive drive;


	public TorGear(Solenoid gearPiston, DigitalInput gearSwitch, Joystick stick, TorDrive drive){
		this.gearPiston = gearPiston;
		this.gearSwitch = gearSwitch;
		this.stick = stick;
		this.drive = drive;
		gearBack = new GearBackTraj();
	}

	public void Gear(){
		if(gearSwitch.get()){
//			drive.shiftToHighGearMotion();
//			TorMotionProfile.INSTANCE.executeTrajectory(gearBack);
			gearOpen();
		}
		else{
			gearClosed();
		}
	}

	public void gearOpen(){
		gearPiston.set(true);
		//determine if true or false for open position
	}

	public void gearClosed(){
		
		gearPiston.set(false);
		//determine if true or false for closed position
	}

	public void update(){
		if(stick.getRawButton(6)){
			gearOpen();
		}
		else{
			gearClosed();
		}
	}
}




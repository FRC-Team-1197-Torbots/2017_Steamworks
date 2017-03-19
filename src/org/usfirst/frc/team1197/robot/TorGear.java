package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class TorGear {

	private Solenoid gearPiston;
	private DigitalInput gearSwitch;
	private Joystick stick;
	private long endTime = System.currentTimeMillis() - 10;
	private long currentTime;

	public TorGear(Solenoid gearPiston, DigitalInput gearSwitch, Joystick stick){
		this.gearPiston = gearPiston;
		this.gearSwitch = gearSwitch;
		this.stick = stick;
	}

	public void autoControl(){
		currentTime = System.currentTimeMillis();
		if(gearSwitch.get()){
			gearOpen();
			endTime = System.currentTimeMillis() + 1500;
		}
		else if(endTime < currentTime){
			gearClosed();
		}
	}

	public void playerControl(){
		if(stick.getRawButton(6)){
			gearOpen();
		}
		else{
			gearClosed();
		}
	}
	
	public boolean gearOn(){
		return gearSwitch.get();
	}

	public void gearOpen(){
		gearPiston.set(true);
	}
	
	public void gearClosed(){
		gearPiston.set(false);
	}
}




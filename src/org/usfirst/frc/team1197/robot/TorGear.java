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
	private boolean isOpen;

	public TorGear(Solenoid gearPiston, DigitalInput gearSwitch, Joystick stick){
		this.gearPiston = gearPiston;
		this.gearSwitch = gearSwitch;
		this.stick = stick;
		isOpen = false;
	}

	public void Gear(){		
		currentTime = System.currentTimeMillis();
		
		
		if(gearSwitch.get() && !isOpen){
			gearOpen();
			isOpen = true;
			endTime = System.currentTimeMillis() + 1500;
		}
		
		if(endTime < currentTime){
			isOpen = false;
			gearClosed();
		}
	}
	
	public boolean gearOn(){
		return gearSwitch.get();
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




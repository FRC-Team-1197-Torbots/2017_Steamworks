package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class TorGear {

	private Solenoid gearPiston;
	private DigitalInput gearSwitch;
	private Joystick stick;


	public TorGear(Solenoid gearPiston, DigitalInput gearSwitch, Joystick stick){
		this.gearPiston = gearPiston;
		this.gearSwitch = gearSwitch;
		this.stick = stick;
	}

	public void Gear(){
		if(gearSwitch.get()){
			gearOpen();
			Timer.delay(0.5);
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




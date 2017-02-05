package org.usfirst.frc.team1197.robot;

import java.util.Set;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;


public class TorClimb {

	private CANTalon climbTalon;
	private DigitalInput climbSwitch;
	private Joystick stick;
	
	public TorClimb(CANTalon climbTalon, DigitalInput climbSwitch, Joystick stick){
		this.climbTalon = climbTalon;
		this.climbSwitch= climbSwitch;
		this.stick = stick;
	}

	public void Climb(){
		if(climbSwitch.get()){
			stopClimb();
		}
		else{
			startClimb();
		}
	}

	public void startClimb(){
		//need to figure out speed
		climbTalon.set(0.8);
	}

	public void stopClimb(){
		climbTalon.set(0.0);
	}
	
	public void update(){
		if(stick.getRawButton(6)){
			startClimb();
		}
		else{
			stopClimb();
		}
	}
}


package org.usfirst.frc.team1197.robot;

import java.util.Set;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;


public class TorClimb {

	private CANTalon climbTalon;
	private DigitalInput climbSwitch;

	public TorClimb(CANTalon climbTalon, DigitalInput climbSwitch){
		this.climbTalon = climbTalon;
		this.climbSwitch= climbSwitch;
	}

	public void Climb(){
		if(climbSwitch.get()){
			stopClimb();
		}
		else{
			climbOn();
		}
	}

	public void climbOn(){
		//need to figure out speed
		climbTalon.set(0.8f);
	}

	public void stopClimb(){
		climbTalon.set(0);
	}
}


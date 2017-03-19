package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class TorClimb {

	private CANTalon climbTalon;
	private Joystick stick;

	public TorClimb(CANTalon climbTalon, Joystick stick){
		this.climbTalon = climbTalon;
		this.stick = stick;
	}
	
	public void playerControl(){
		if(stick.getRawButton(6)){
			climbTalon.set(1.0); //- for final, + for proto
		}
		else{
			climbTalon.set(0.0);
		}
	}
}



package org.usfirst.frc.team1197.robot;

import java.util.Set;

import com.ctre.CANTalon;


public class TorClimb {

	private CANTalon climbTalon;

	public TorClimb(CANTalon climbTalon){
		this.climbTalon = climbTalon;
	}

	public void Climb(){
		//need to figure out speed
		climbTalon.set(0.8f);

	}

	public void StopClimb(){

		climbTalon.set(0);
		
	}

}


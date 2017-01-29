package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;

public class TorIntake {

	private CANTalon elevatorTalon;
	private CANTalon dumperTalon;

	public TorIntake(CANTalon elevatorTalon, CANTalon dumperTalon){
		this.elevatorTalon = elevatorTalon;
		this.dumperTalon = dumperTalon;

	}

	public void IntakeIn(){
		elevatorTalon.set(0.6);
		dumperTalon.set(0.6);
		//going in same direction
	}
	
	public void IntakeOut(){
		elevatorTalon.set(-0.6);
		dumperTalon.set(0.6);
		//elevator going in opposite direction
	}

	public void Dumper(){
		elevatorTalon.set(0.6);
		dumperTalon.set(-0.6);
		//dumper going in opposite direction
	}
	
	public void IntakeOff(){
		elevatorTalon.set(0);
		dumperTalon.set(0);
	}
}


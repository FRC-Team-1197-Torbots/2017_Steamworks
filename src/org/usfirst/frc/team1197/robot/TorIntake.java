package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;

/**
 * 2017 Intake.  Features a intake vertical elevator and
 * single roller bar for pushing balls out of the hopper.
 * 
 * No sensors required
 * @author Torbot
 *
 */
public class TorIntake {

	private CANTalon elevatorTalon;
	private CANTalon dumperTalon;

	public TorIntake(CANTalon elevatorTalon, CANTalon dumperTalon){
		this.elevatorTalon = elevatorTalon;
		this.dumperTalon = dumperTalon;
	}

	/**
	 * Run the intake elevator and dumper in the same direction
	 * Both have to be running to keep balls inside the hopper
	 */
	public void IntakeIn(){
		elevatorTalon.set(0.6);
		dumperTalon.set(0.6);
		//going in same direction
	}
	
	/**
	 * Pushes balls back onto the floor.  Run the elevator in reverse
	 * and keep the dumper wheel spinning to keep balls inside
	 */
	public void IntakeOut(){
		elevatorTalon.set(-0.6);
		dumperTalon.set(0.6);
		//elevator going in opposite direction
	}

	/**
	 * Function pushes balls out of the hopper.
	 * Must run both motors because elevator needs to run in reverse
	 * so balls do not get forced back down elevator
	 */
	public void DumpBalls(){
		elevatorTalon.set(0.6);
		dumperTalon.set(-0.6);
		//dumper going in opposite direction
	}
	
	/**
	 * Function to turn off intake
	 */
	public void IntakeOff(){
		elevatorTalon.set(0);
		dumperTalon.set(0);
	}
}


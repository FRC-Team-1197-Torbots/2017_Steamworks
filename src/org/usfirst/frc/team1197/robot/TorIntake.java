package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Joystick;

/**
 * 2017 Intake.  Features a intake vertical elevator and
 * single roller bar for pushing balls out of the hopper.
 * 
 * No sensors required
 * @author Torbot
 *
 */
public class TorIntake {

	private CANTalon elevatorTalon1;
	private CANTalon elevatorTalon2;
	private CANTalon dumperTalon;
	private Joystick stick;

	public TorIntake(CANTalon elevatorTalon1, CANTalon elevatorTalon2, CANTalon dumperTalon, Joystick stick){
		this.elevatorTalon1 = elevatorTalon1;
		this.elevatorTalon2 = elevatorTalon2;
		this.dumperTalon = dumperTalon;
		this.stick = stick;
	}

	/**
	 * Run the intake elevator and dumper in the same direction
	 * Both have to be running to keep balls inside the hopper
	 */
	public void IntakeIn(){
		elevatorTalon1.set(0.8); //-final +proto
		elevatorTalon2.set(0.8);
		dumperTalon.set(0.5);
		//going in same direction
	}

	/**
	 * Pushes balls back onto the floor.  Run the elevator in reverse
	 * and keep the dumper wheel spinning to keep balls inside
	 */
	public void elevatorIn(){
		elevatorTalon1.set(0.8);
		elevatorTalon2.set(0.8);
		dumperTalon.set(0.5);
		//elevator going in opposite direction
	}

	/**
	 * Function pushes balls out of the hopper.
	 * Must run both motors because elevator needs to run in reverse
	 * so balls do not get forced back down elevator
	 */
	public void DumpBalls(){
		elevatorTalon1.set(1.0);
		elevatorTalon2.set(1.0);
		dumperTalon.set(-1.0);
		//dumper going in opposite direction
	}

	/**
	 * Function to turn off intake
	 */
	public void IntakeOff(){
		elevatorTalon1.set(0.0);
		elevatorTalon2.set(0.0);
		dumperTalon.set(0.0);
	}
	

	public void update(){
		if(stick.getRawButton(1)){
			IntakeIn();
		}
		else if(stick.getRawButton(2)){
			elevatorIn();
		}
		else if(stick.getRawButton(3)){
			DumpBalls();
		}
		else{
			IntakeOff();
		}
	}
}


package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class TorGearIntake {

	private Solenoid gearIntakePiston;
	private CANTalon gearIntake;
	private Joystick stick;

	public TorGearIntake(Joystick stick, CANTalon gearIntake, Solenoid gearIntakePiston){
		this.gearIntake = gearIntake;
		this.gearIntakePiston = gearIntakePiston;
	}
	
	public void playerControl(){
		if(stick.getRawButton(5)){
			gearIntakePiston.set(true);
		}
		else{
			gearIntakePiston.set(false);
		}
		if(stick.getRawButton(1)){
			gearIntake.set(1.0);
		}
		else if(stick.getRawButton(2)){
			gearIntake.set(-1.0);
		}
		else{
			gearIntake.set(0.0);
		}
	}

}


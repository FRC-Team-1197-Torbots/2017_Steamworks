package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;


public class TorClimb {

	private CANTalon climbTalon;
	private DigitalInput climbSwitch;
	private Joystick stick;

	public TorClimb(CANTalon climbTalon, Joystick stick){
		this.climbTalon = climbTalon;
		this.stick = stick;
	}

	public void switchClimb(){
		if(climbSwitch.get()){
			stopClimb();
		}
		else{
			startClimb();
		}
	}
	
	public void manualClimb(){
		if(stick.getRawButton(6)){
			climbTalon.set(1.0); //- for final, + for proto
		}
		else{
			climbTalon.set(0.0);
		}
	}
	
	public void startClimb(){
		climbTalon.set(1.0);
	}
	
	public void reverseClimb(){
		climbTalon.set(-1.0);
	}

	public void stopClimb(){
		climbTalon.set(0.0);
	}
}



package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;


public class TorClimb {

	private CANTalon climbTalon;
	private DigitalInput climbSwitch;
	private Joystick stick;
	private boolean climbFlag = true;
	private boolean overrideFlag = false;

	public TorClimb(CANTalon climbTalon, DigitalInput climbSwitch, Joystick stick){
		this.climbTalon = climbTalon;
		this.climbSwitch= climbSwitch;
		this.stick = stick;
	}

	public void climb(){
		if(climbSwitch.get()){
			stopClimb();
			climbFlag = false;
		}
		else if(climbFlag){
			startClimb();
		}
	}
	
	public void overrideClimb(){
		if(overrideFlag && !stick.getRawButton(6)) {
			stopClimb();
			//if the override button and the climbing button are pressed, start climb
		}
		else if(overrideFlag && stick.getRawButton(6)){
			startClimb();
		}
	}

	public void startClimb(){
		climbTalon.set(1.0);
	}

	public void stopClimb(){
		climbTalon.set(0.0);
	}

	public void update(){
		if(overrideFlag){
			overrideClimb();
		}
		else if(stick.getRawButton(5)){
			climb();
		}
		else{
			override();
			stopClimb();
		}
	}

	public void override(){
		//changes overrideFlag to true if the button is pressed
		if(stick.getRawButton(8)){
			overrideFlag = true;
		}
	}
	
	public void resetClimb(){
		climbFlag = true;
		overrideFlag = false;
	}
}



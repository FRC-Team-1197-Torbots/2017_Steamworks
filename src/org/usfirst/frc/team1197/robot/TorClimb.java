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
	private double finalDist;
	private double initDist;
	private double error = finalDist - initDist;
	private double speed = 1.0;
	private TorLidar lidar;
	private TorDrive drive;

	public TorClimb(CANTalon climbTalon, DigitalInput climbSwitch, Joystick stick, TorLidar lidar, TorDrive drive){
		this.climbTalon = climbTalon;
		this.climbSwitch= climbSwitch;
		this.stick = stick;
		this.lidar = lidar;
		this.drive = drive;
	}

	public void switchClimb(){
		if(climbSwitch.get()){
			stopClimb();
			climbFlag = false;
		}
		else if(climbFlag){
			startClimb();
		}
	}
	
	public void lidarClimb(){
		if(error < (finalDist * 0.25)){
			climbTalon.set(speed * 0.25);
		}
		else if(error < (finalDist * 0.5)){
			climbTalon.set(speed * 0.5);
		}
		else if(error < (finalDist * 0.75)){
			climbTalon.set(speed * 0.75);
		}
		else{
			climbTalon.set(speed);
		}
	}
	
	public void overrideClimb(){
		if(overrideFlag && stick.getRawButton(6)){
			startClimb();
		}
		else if(overrideFlag && stick.getRawButton(5)){
			reverseClimb();
		}
		else{
			stopClimb();
		}
	}

	public void update(){
		if(overrideFlag){
			overrideClimb();
		}
		else{
			switchClimb();
			override();
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



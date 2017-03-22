package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class TorGearIntake {

	private Solenoid gearIntakePiston;
	private CANTalon gearIntake;
	private Joystick stick;
	private DigitalInput gearIntakeDetector;
	private long endTime = System.currentTimeMillis() - 10;
	private long currentTime;
	private boolean firstTime = true;

	public TorGearIntake(Joystick stick, CANTalon gearIntake, Solenoid gearIntakePiston, DigitalInput gearIntakeDetector){
		this.stick = stick;
		this.gearIntake = gearIntake;
		this.gearIntakePiston = gearIntakePiston;
		this.gearIntakeDetector = gearIntakeDetector;
	}
	
	public void autoControl(){
		currentTime = System.currentTimeMillis();
		if(stick.getRawButton(5)){
			gearIntakePiston.set(true);
			if(!gearIntakeDetector.get()){
				if(firstTime){
					endTime = System.currentTimeMillis() + 1000;
					firstTime = false;
				}
				if(endTime < currentTime){
					gearIntake.set(0.0);
				}
				else{
					gearIntake.set(-0.5);
				}
			}
			else{
				firstTime = true;
				gearIntake.set(-1.0);
			}
		}
		else if(stick.getRawButton(3)){
			gearIntakePiston.set(true);
			if(!gearIntakeDetector.get()){
				gearIntake.set(1.0);

			}
		}
		else{
			gearIntakePiston.set(false);
			if(endTime < currentTime){
				gearIntake.set(0.0);
			}
			else{
				gearIntake.set(-0.5);
			}
		}
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
		else if(stick.getRawButton(3)){
			gearIntake.set(-1.0);
		}
		else{
			gearIntake.set(0.0);
		}
	}

}


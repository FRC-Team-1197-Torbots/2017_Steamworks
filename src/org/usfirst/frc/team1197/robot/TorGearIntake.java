package org.usfirst.frc.team1197.robot;

import org.usfirst.frc.team1197.robot.TorAuto.BOILERAUTO;

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
	private double threshold = 10.0;
	private boolean justAcquired = false;
	
	public static enum GEARINTAKE
	{
		RETRACTED, ACQUIRING, DEPLOYING; 

		private GEARINTAKE() {}
	}
	public GEARINTAKE gearIntakeState = GEARINTAKE.RETRACTED;

	public TorGearIntake(Joystick stick, CANTalon gearIntake, Solenoid gearIntakePiston, DigitalInput gearIntakeDetector){
		this.stick = stick;
		this.gearIntake = gearIntake;
		this.gearIntakePiston = gearIntakePiston;
		this.gearIntakeDetector = gearIntakeDetector;
	}
	
	public void autoControl(){
		currentTime = System.currentTimeMillis();
//		if(stick.getRawButton(5)){
//			gearIntakePiston.set(true);
//			if(!gearIntakeDetector.get()){
//				if(firstTime){
//					endTime = System.currentTimeMillis() + 1000;
//					firstTime = false;
//				}
//				if(endTime < currentTime){
//					gearIntake.set(0.0);
//				}
//				else{
//					gearIntake.set(-0.5);
//				}
//			}
//			else{
//				firstTime = true;
//				gearIntake.set(-1.0);
//			}
//		}
//		else if(stick.getRawButton(3)){
//			gearIntakePiston.set(true);
//			if(!gearIntakeDetector.get()){
//				gearIntake.set(1.0);
//			}
//		}
//		else{
//			gearIntakePiston.set(false);
//			if(endTime < currentTime){
//				gearIntake.set(0.0);
//			}
//			else{
//				gearIntake.set(-0.5);
//			}
//		}
		switch(gearIntakeState){
		case RETRACTED:
			if(!stick.getRawButton(2)){
				justAcquired = false;
			}
			if(currentTime > endTime){
				gearIntake.set(0.0);
			}
			if(!justAcquired && stick.getRawButton(2)){
				setStateAcquiring();
			}
			else if(stick.getRawButton(3)){
				setStateDeploying();
			}
			break;
		case ACQUIRING:
			if(!stick.getRawButton(2)){
				setStateRetracted();
				endTime = -10;
			}
			if(Math.abs(gearIntake.getOutputCurrent()) > threshold){
				if(firstTime){
					endTime = System.currentTimeMillis() + 300;
					firstTime = false;
				}
			}
			if(currentTime > endTime){
				if(Math.abs(gearIntake.getOutputCurrent()) > threshold){
					endTime = System.currentTimeMillis() + 1000;
					justAcquired = true;
					setStateRetracted();
				}
				else{
					firstTime = true;
				}
			}
			break;
		case DEPLOYING:
			if(!stick.getRawButton(3)){
				setStateRetracted();
				endTime = -10;
			}
			break;
		}
	}
	
	public void setStateRetracted(){
		gearIntakePiston.set(false);
		gearIntakeState = GEARINTAKE.RETRACTED;
	}
	
	public void setStateAcquiring(){
		endTime = System.currentTimeMillis() + 150000;
		firstTime = true;
		gearIntakePiston.set(true);
		gearIntake.set(-1.0);
		gearIntakeState = GEARINTAKE.ACQUIRING;
	}
	
	public void setStateDeploying(){
		firstTime = true;
		gearIntakePiston.set(true);
		gearIntake.set(0.5);
		gearIntakeState = GEARINTAKE.DEPLOYING;
	}
	
	public void enable(){
		setStateRetracted();
		endTime = System.currentTimeMillis() - 10;
	}

	
	public void playerControl(){
		if(stick.getRawButton(2)){
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


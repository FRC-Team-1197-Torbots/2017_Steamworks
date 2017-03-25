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
	
	public void autoControl(boolean acquireButton, boolean deployButton){
		currentTime = System.currentTimeMillis();
		switch(gearIntakeState){
		case RETRACTED:
			if(!acquireButton){
				justAcquired = false;
			}
			if(currentTime > endTime){
				gearIntake.set(0.0);
			}
			if(!justAcquired && acquireButton){
				setStateAcquiring();
			}
			else if(deployButton){
				setStateDeploying();
			}
			break;
		case ACQUIRING:
			if(!deployButton){
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
					System.out.println("RETRACTING");
				}
				else{
					firstTime = true;
				}
			}
			break;
		case DEPLOYING:
			if(!deployButton){
				setStateRetracted();
				endTime = -10;
			}
			break;
		}
	}
	
	public void setStateRetracted(){
//		System.out.println("RETRACTED");
		gearIntakePiston.set(false);
		gearIntakeState = GEARINTAKE.RETRACTED;
	}
	
	public void setStateAcquiring(){
//		System.out.println("ACQUIRING");
		endTime = System.currentTimeMillis() + 150000;
		firstTime = true;
		gearIntakePiston.set(true);
		gearIntake.set(-1.0);
		gearIntakeState = GEARINTAKE.ACQUIRING;
	}
	
	public void setStateDeploying(){
//		System.out.println("DEPLOYING");
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


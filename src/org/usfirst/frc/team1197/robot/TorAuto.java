package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private int position; //position goes from left to right
	private TorIntake intake;
	private Joystick cypress;
	
	private BoilerPos1 boilerPos1;
	private BoilerPos2 boilerPos2;
	private BoilerPos3 boilerPos3;
	
	private TorGear gear;
	
	private GearBackTraj gearback;
	
	public static enum BOILERAUTO
	{
		IDLE, POS0, POS1, POS2, POS3, POS4;

		private BOILERAUTO() {}
	}
	public BOILERAUTO boilerAutoState = BOILERAUTO.IDLE;
	
	public static enum LOADSTATIONAUTO
	{
		IDLE, POS0, POS1, POS2;
		
		private LOADSTATIONAUTO() {}
	}
	public LOADSTATIONAUTO loadAutoState = LOADSTATIONAUTO.IDLE;
	
	public static enum CENTERAUTO
	{
		IDLE, POS0, POS1, POS2, POS3, POS4;
		
		private CENTERAUTO() {}
	}
	public CENTERAUTO centerAutoState = CENTERAUTO.IDLE;
	
	public TorAuto(TorIntake intake, Joystick cypress, TorGear gear) {
		this.cypress = cypress;
		boilerPos1 = new BoilerPos1();
		boilerPos2 = new BoilerPos2();
		boilerPos3 = new BoilerPos3();
		gearback = new GearBackTraj();
		this.intake = intake;
		this.gear = gear;
	}
	
	public void initialize()
	{
		//determine the position of the robot
		if(!cypress.getRawButton(1)  && cypress.getRawButton(2)){
			position = 1;
		}
		else if(!cypress.getRawButton(1) && !cypress.getRawButton(2)){
			position = 2;
		}
		else if(cypress.getRawButton(1) && !cypress.getRawButton(2)){
			position = 3;
		}
		else{
			position = -1;
		}
		
	}
	
	public void run() {
		//use position to determine which auto to run
		if(position < 0) {
			//something bad
		}
		else if(position == 1){
			boiler();
		}
		else if(position == 2){
			center();
		}
		else if(position == 3){
			loadStation();
		}
	}
	
	/*****************************************************
	 * BELOW ARE THE THREE AUTO CODE SITUATIONS
	 *****************************************************/
	
	public void center() {
		centerAutoState = CENTERAUTO.POS0;
		while(centerAutoState != CENTERAUTO.IDLE){
			switch(centerAutoState){
			case IDLE:
				
				break;
			case POS0:
				
				break;
			case POS1:
				
				break;
			case POS2:
				
				break;
			case POS3:
				break;
			}
		}
	}
	
	public void loadStation() {
		loadAutoState = LOADSTATIONAUTO.POS0;
		while(loadAutoState != LOADSTATIONAUTO.IDLE){
			switch(loadAutoState){
			case IDLE:
				
				break;
			case POS0:
				
				break;
			case POS1:
				
				break;
			case POS2:
				
				break;
			}
		}
	}
	
	public void boiler() {
		boilerAutoState = BOILERAUTO.POS0;
		while(boilerAutoState != BOILERAUTO.IDLE){
			switch(boilerAutoState){
			case IDLE:
				break;
			case POS0:
				TorMotionProfile.INSTANCE.executeTrajectory(boilerPos1);
				boilerAutoState = BOILERAUTO.POS1;
				break;
			case POS1:
				gear.Gear();
				if(boilerPos1.isComplete()){
					//deploy gear
					TorMotionProfile.INSTANCE.executeTrajectory(gearback);
					boilerAutoState = BOILERAUTO.POS2;	
				}
				break;
			case POS2:
				if(gearback.isComplete()){
					//dump balls
//					TorMotionProfile.INSTANCE.executeTrajectory(boilerPos3);
					boilerAutoState = BOILERAUTO.POS3;
				}
			case POS3:
//				if(boilerPos3.isComplete()){
					boilerAutoState = BOILERAUTO.IDLE;
//				}
				break;
			}
		}
	}
}
package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private int position; //position goes from left to right
	private TorDrive drive;
	private Joystick cypress;
	
	private BoilerPos1 boilerPos1;
	private BoilerPos2 boilerPos2;
	private BoilerPos3 boilerPos3;
	
	private LoadingPos1 loadingPos1;
	private LoadingPos2 loadingPos2;
	private LoadingPos3 loadingPos3;
	
	private CenterPos1 centerPos1;
	private CenterPos2 centerPos2;
	
	private TwoGearPos1 twoGearPos1;
	private TwoGearPos2 twoGearPos2;
	private TwoGearPos3 twoGearPos3;
	private TwoGearPos4 twoGearPos4;
	
	private TorGear gear;
	
	public static enum BOILERAUTO
	{
		IDLE, POS0, POS1, POS2, POS3;

		private BOILERAUTO() {}
	}
	public BOILERAUTO boilerAutoState = BOILERAUTO.IDLE;
	
	public static enum LOADSTATIONAUTO
	{
		IDLE, POS0, POS1, POS2, POS3;
		
		private LOADSTATIONAUTO() {}
	}
	public LOADSTATIONAUTO loadAutoState = LOADSTATIONAUTO.IDLE;
	
	public static enum CENTERAUTO
	{
		IDLE, POS0, POS1, POS2;
		
		private CENTERAUTO() {}
	}
	public CENTERAUTO centerAutoState = CENTERAUTO.IDLE;
	
	public static enum TWOGEARAUTO
	{
		IDLE, POS0, ZIGZAGGING, DRIVINGBACK, TURNINGAROUND, DRIVINGFORWARD;
		
		private TWOGEARAUTO() {}
	}
	public TWOGEARAUTO twoGearAutoState = TWOGEARAUTO.IDLE;	
	
	public TorAuto(TorDrive drive, Joystick cypress, TorGear gear) {
		this.drive = drive;
		this.cypress = cypress;
		
		boilerPos1 = new BoilerPos1();
		boilerPos2 = new BoilerPos2();
		boilerPos3 = new BoilerPos3();
		
		loadingPos1 = new LoadingPos1();
		loadingPos2 = new LoadingPos2();
		loadingPos3 = new LoadingPos3();
		
		centerPos1 = new CenterPos1();
		centerPos2 = new CenterPos2();
		
		this.gear = gear;
	}
	
	public void initialize()
	{
		//determine the position of the robot
		if(!cypress.getRawButton(3)  && cypress.getRawButton(2)){
			position = 1;
		}
		else if(!cypress.getRawButton(3) && !cypress.getRawButton(2)){
			position = 2;
		}
		else if(cypress.getRawButton(3) && !cypress.getRawButton(2)){
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
				drive.executeTrajectory(centerPos1);
				centerAutoState = CENTERAUTO.POS1;
				break;
			case POS1:
				gear.autoControl();
				if(centerPos1.isComplete()){
					drive.executeTrajectory(centerPos2);
					centerAutoState = CENTERAUTO.POS2;
				}
				break;
			case POS2:
				if(centerPos2.isComplete()){
					centerAutoState = CENTERAUTO.IDLE;
				}
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
				drive.executeTrajectory(loadingPos1);
				loadAutoState = LOADSTATIONAUTO.POS1;
				break;
			case POS1:
				gear.autoControl();
				if(loadingPos1.isComplete()){
					drive.executeTrajectory(loadingPos2);
					loadAutoState = LOADSTATIONAUTO.POS2;
				}
				break;
			case POS2:
				if(loadingPos2.isComplete()){
//					drive.executeTrajectory(loadingPos3);
					loadAutoState = LOADSTATIONAUTO.IDLE;
				}
				break;
			case POS3:
				if(loadingPos3.isComplete()){
					loadAutoState = LOADSTATIONAUTO.IDLE;
				}
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
				drive.executeTrajectory(boilerPos1);
				boilerAutoState = BOILERAUTO.POS1;
				break;
			case POS1:
				gear.autoControl();
				if(boilerPos1.isComplete()){
					drive.executeTrajectory(boilerPos2);
					boilerAutoState = BOILERAUTO.POS2;	
				}
				break;
			case POS2:
				if(boilerPos2.isComplete()){
//					drive.executeTrajectory(boilerPos3);
					boilerAutoState = BOILERAUTO.IDLE;
				}
				break;
			case POS3:
				if(boilerPos3.isComplete()){
					boilerAutoState = BOILERAUTO.IDLE;
				}
				break;
			}
		}
	}
	
	public void twoGear() {
		twoGearAutoState = TWOGEARAUTO.POS0;
		while(twoGearAutoState != TWOGEARAUTO.IDLE){
			gear.autoControl(); // put in button values
			switch(twoGearAutoState){
			case IDLE:
				break;
			case POS0:
				drive.executeTrajectory(twoGearPos1);
				twoGearAutoState = TWOGEARAUTO.ZIGZAGGING;
				break;
			case ZIGZAGGING:
				if(twoGearPos1.isComplete()){
				//drop the gear
				//set intake to "acquire"
					drive.executeTrajectory(twoGearPos2);
					twoGearAutoState = TWOGEARAUTO.DRIVINGBACK;
				}
				break;
			case DRIVINGBACK:
				if(twoGearPos2.isComplete()){ 
					drive.executeTrajectory(twoGearPos3);
					twoGearAutoState = TWOGEARAUTO.TURNINGAROUND;
				}
				break;
			case TURNINGAROUND:
				if(twoGearPos3.isComplete()){ 
				// "let go" of virtual intake button to retract
//					drive.executeTrajectory(twoGearPos4);
					twoGearAutoState = TWOGEARAUTO.DRIVINGFORWARD;
				}
				break;
			case DRIVINGFORWARD:
				if(twoGearPos4.isComplete()){ 
					// deploy gear
					// maybe have one more trajectory to back up a little bit?
					boilerAutoState = BOILERAUTO.IDLE;
				}
				break;
			}
			
		}
	}
}
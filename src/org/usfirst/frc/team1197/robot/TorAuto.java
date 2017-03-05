package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {
	private int position; //position goes from left to right
	private TorIntake intake;
	private Joystick cypress;
	
	private BoilerPos1 boilerPos1;
	private BoilerPos2 boilerPos2;
	private BoilerPos3 boilerPos3;
	
	private LoadingPos1 loadingPos1;
	private LoadingPos2 loadingPos2;
	private LoadingPos3 loadingPos3;
	
	private CenterPos1 centerPos1;
	private CenterPos2 centerPos2;
	
	private TorGear gear;
	private GearBackTraj gearback;
	
	private long currentTime;
	private long endTime = System.currentTimeMillis() - 10;
	
	public static enum BOILERAUTO
	{
		IDLE, POS0, POS1, POS2, POS3, POS4, POS5;

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
		IDLE, POS0, POS1, POS2, POS3, POS4;
		
		private CENTERAUTO() {}
	}
	public CENTERAUTO centerAutoState = CENTERAUTO.IDLE;
	
	public TorAuto(TorIntake intake, Joystick cypress, TorGear gear) {
		this.cypress = cypress;
		
		boilerPos1 = new BoilerPos1();
		boilerPos2 = new BoilerPos2();
		boilerPos3 = new BoilerPos3();
		
		loadingPos1 = new LoadingPos1();
		loadingPos2 = new LoadingPos2();
		loadingPos3 = new LoadingPos3();
		
		centerPos1 = new CenterPos1();
		centerPos2 = new CenterPos2();
		
		gearback = new GearBackTraj();
		this.intake = intake;
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
				TorMotionProfile.INSTANCE.executeTrajectory(centerPos1);
				centerAutoState = CENTERAUTO.POS1;
				break;
			case POS1:
				gear.Gear();
				if(centerPos1.isComplete()){
					TorMotionProfile.INSTANCE.executeTrajectory(centerPos2);
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
				TorMotionProfile.INSTANCE.executeTrajectory(loadingPos1);
				loadAutoState = LOADSTATIONAUTO.POS1;
				break;
			case POS1:
				gear.Gear();
				if(loadingPos1.isComplete()){
					TorMotionProfile.INSTANCE.executeTrajectory(loadingPos2);
					loadAutoState = LOADSTATIONAUTO.POS2;
				}
				break;
			case POS2:
				if(loadingPos2.isComplete()){
					TorMotionProfile.INSTANCE.executeTrajectory(loadingPos3);
					loadAutoState = LOADSTATIONAUTO.POS3;
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
				TorMotionProfile.INSTANCE.executeTrajectory(boilerPos1);
				boilerAutoState = BOILERAUTO.POS1;
				break;
			case POS1:
				gear.Gear();
				if(boilerPos1.isComplete()){
					//deploy gear
					TorMotionProfile.INSTANCE.executeTrajectory(boilerPos2);
					boilerAutoState = BOILERAUTO.POS2;	
				}
				break;
			case POS2:
				if(boilerPos2.isComplete()){
					endTime = System.currentTimeMillis() + 1500;
//					TorMotionProfile.INSTANCE.executeTrajectory(boilerPos3);
					boilerAutoState = BOILERAUTO.POS3;
				}
				break;
			case POS3:
				currentTime = System.currentTimeMillis();
				intake.DumpBalls();
				if(endTime < currentTime){
					intake.IntakeOff();
					TorMotionProfile.INSTANCE.executeTrajectory(boilerPos3);
					boilerAutoState = BOILERAUTO.POS4;
				}
				break;
			case POS4:
				if(boilerPos3.isComplete()){
					boilerAutoState = BOILERAUTO.IDLE;
				}
				break;
			}
		}
	}
}
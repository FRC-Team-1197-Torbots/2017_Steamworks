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
		boilerAutoState = BOILERAUTO.POS0;
		loadAutoState = LOADSTATIONAUTO.POS0;
		centerAutoState = CENTERAUTO.POS0;
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
	
	
	public void loadStation() {
			switch(loadAutoState){
			case IDLE:
				break;
			case POS0:
				drive.executeTrajectory(loadingPos1);
//				System.out.println("PosError: " + drive.controller.getPositionError() + " VelError: " + drive.controller.getVelocityError());
//				System.out.println("HeadError: " + drive.controller.getHeadingError() + " OmgError: " + drive.controller.getOmegaError());
//				System.out.println("-----------------------------------------------------------------------------------------------------");
				loadAutoState = LOADSTATIONAUTO.POS1;
				break;
			case POS1:
				gear.autoControl();
				if(loadingPos1.isComplete()){
					drive.executeTrajectory(loadingPos2);
//					System.out.println("PosError: " + drive.controller.getPositionError() + " VelError: " + drive.controller.getVelocityError());
//					System.out.println("HeadError: " + drive.controller.getHeadingError() + " OmgError: " + drive.controller.getOmegaError());
//					System.out.println("-----------------------------------------------------------------------------------------------------");
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
	
	
	public void boiler() {
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
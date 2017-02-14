package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.Joystick;

public class TorAuto {

	private int position; //position goes from left to right
	private TorIntake intake;
	private TorDrive drive;
	private Joystick cypress;
	
	public static enum BOILERAUTO
	{
		IDLE, POS1, POS2, POS3, POS4;

		private BOILERAUTO() {}
	}
	public BOILERAUTO boilerAutoState = BOILERAUTO.IDLE;
	
	public TorAuto(TorIntake intake, TorDrive drive, Joystick cypress) {
		this.intake = intake;
		this.cypress = cypress;
		this.drive = drive;
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
			left();
		}
		else if(position == 2){
			center();
		}
		else if(position == 3){
			right();
		}
	}
	
	/*****************************************************
	 * BELOW ARE THE THREE AUTO CODE SITUATIONS
	 *****************************************************/
	
	public void center() {
		//drive to gear 
		
		//drive past baseline
	}
	
	public void right() {
		//drive to gear

//		intake.DumpBalls();
		
		//drive to boiler
		
		//drive past baseline
	}
	
	public void left() {
		boilerAutoState = BOILERAUTO.POS1;
		while(boilerAutoState != BOILERAUTO.IDLE){
			switch(boilerAutoState){
			case IDLE:
				break;
			case POS1:
				drive.autoTrajExecutor(1);
				if(drive.autoTrajChecker(1)){
					boilerAutoState = BOILERAUTO.POS2;	
				}
				break;
			case POS2:
				drive.autoTrajExecutor(2);
				if(drive.autoTrajChecker(2)){
					boilerAutoState = BOILERAUTO.IDLE;
				}
				break;
			}
		}
	}
}
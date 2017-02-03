package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class TorGear {

	private Solenoid gearPiston;
	private DigitalInput gearSwitch;


	public TorGear(Solenoid gearPiston, DigitalInput climbSwitch){
		this.gearPiston = gearPiston;
		this.gearSwitch = climbSwitch;
	}

	public void Gear(){
		if(gearSwitch.get()){
			gearOpen();
		}
		else{
			gearClosed();
		}
	}
	
	public void gearOpen(){
		gearPiston.set(true);
		//determine if true or false for open position
	}
	
	public void gearClosed(){
		gearPiston.set(false);
		//determine if true or false for closed position
	}

}



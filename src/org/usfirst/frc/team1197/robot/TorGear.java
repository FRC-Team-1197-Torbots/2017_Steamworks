package org.usfirst.frc.team1197.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class TorGear {

	private Solenoid gearPiston;
	private DigitalInput limitSwitch;


	public TorGear(Solenoid gearPiston, DigitalInput limitSwitch){
		this.gearPiston = gearPiston;
		this.limitSwitch = limitSwitch;
	}

	public void Gear(){
		if(limitSwitch.get()){
			gearPiston.set(true);
			//need to determine if true or false
		}else
			gearPiston.set(false);
	}

}



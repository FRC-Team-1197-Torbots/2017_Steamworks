package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;


import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public enum TorCAN
{
	INSTANCE;
	
	private final CANTalon rightMaster;
	private final CANTalon rightSlave1;
	private final CANTalon rightSlave2;
	private final CANTalon leftMaster;
	private final CANTalon leftSlave1;
	private final CANTalon leftSlave2;
	
	private final AHRS gyro;
	private final double encoderTicksPerMeter = 7110.6; // (units: ticks per meter) 
	private final double approximateSensorSpeed = 4357; // measured maximum (units: RPM)
	private final double quadEncNativeUnits = 512.0; // (units: ticks per revolution)
	private final double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));

	private final double kP = 1.5; //1.5
	private final double kI = 0.0; //0.0
	private final double kD = 30.0; //30.0

	// absoluteMaxSpeed is in meters per second. Right now it comes out to about 4.405 m/s
	private final double absoluteMaxSpeed = (approximateSensorSpeed*quadEncNativeUnits)/(60*encoderTicksPerMeter);
	private final double trackWidth = 0.5786; // (units: meters (14.5 in inches))
	private final double halfTrackWidth = trackWidth / 2.0; // (units: meters)
	
	private final double backlash = 0.015; // (units: meters)
	
	private TorCAN(){
		gyro = new AHRS(SerialPort.Port.kMXP);

		leftSlave1 = new CANTalon(7); //7
		leftMaster = new CANTalon(8); //8
		leftSlave2 = new CANTalon(9); //9
		rightSlave1 = new CANTalon(1); //1
		rightMaster = new CANTalon(2); //2
		rightSlave2 = new CANTalon(3); //3

		//use only when testing to get approximate sensor speed
//		rightSlave2.configEncoderCodesPerRev(128);
//		leftMaster.configEncoderCodesPerRev(128);

		rightMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightMaster.reverseSensor(false); //don't forget to change! false - final, true - proto
		rightMaster.reverseOutput(false); //don't forget to change! false - final, false - proto
		rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		rightMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		rightMaster.setProfile(0);
		rightMaster.setF(kF); 
		rightMaster.setP(kP); 
		rightMaster.setI(kI); 
		rightMaster.setD(kD);

		rightSlave1.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightSlave1.set(rightMaster.getDeviceID());
		rightSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightSlave2.set(rightMaster.getDeviceID());
		
		leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		leftMaster.reverseSensor(true); //don't forget to change! true - final, false - proto
		leftMaster.reverseOutput(true); //don't forget to change! true - final, true - proto
		leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		leftMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftMaster.setProfile(0);
		leftMaster.setF(kF);  
		leftMaster.setP(kP); 
		leftMaster.setI(kI); 
		leftMaster.setD(kD); 
		
		leftSlave1.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftSlave1.set(leftMaster.getDeviceID());
		leftSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftSlave2.set(leftMaster.getDeviceID());

		rightMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
		leftMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
	}

	public void SetDrive(double leftSpeed, double rightSpeed)
	{
		SetLeft(-leftSpeed);
		SetRight(rightSpeed);
	}
	
	//Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed)
	{
		leftMaster.set(speed);
	}

	//Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed)
	{
		rightMaster.set(speed);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to percentVbus, so we can use it when the robot is in low gear. */
	public void choosePercentVbus(){
		rightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		leftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to speed, so we can use it when the robot is in high gear. */
	public void chooseVelocityControl(){
		rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
		leftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
	}
	
	public void chooseMotionProfileControl(){
		rightMaster.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
		leftMaster.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
	}
	
	public double getRightEncoder(){
		return rightMaster.getPosition();
	}
	public double getLeftEncoder(){
		return leftMaster.getPosition();
	}
	
	public double getAverageRawVelocity(){
		return (rightMaster.getSpeed() + leftMaster.getSpeed()) * 0.5;
	}
	public double getAverageEncoderPosition(){
		return (rightMaster.getPosition() + leftMaster.getPosition()) * 0.5;
	}
	public double getPosition(){
		return (rightMaster.getPosition() + leftMaster.getPosition()) * 0.5 / encoderTicksPerMeter; // (units: meters)
	}
	public double getVelocity(){
		return (rightMaster.getSpeed() + leftMaster.getSpeed()) * 0.5 * 10 / encoderTicksPerMeter; // (units: meters)
	}
	
	public double getHeading(){
		return (-gyro.getAngle() * (Math.PI / 180)); // (units: radians)
	}
	public double getOmega(){
		return (-gyro.getRate()); // (units: radians (contrary to navX documentation))
	}
	
	public void setTargets(double v, double omega){ 
		rightMaster.set((v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		leftMaster.set((v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);		
	}
	
	public void resetEncoder(){
		rightMaster.setPosition(0);
		leftMaster.setPosition(0);
	}
	
	public void resetHeading(){
		gyro.reset();
	}
	
	public double getBacklash(){
		return backlash; // (units: meters)
	}
	
	public double absoluteMaxSpeed(){
		return absoluteMaxSpeed; // (units: meters per second)
	}
	
	public double getSensorSpeed(){
		return approximateSensorSpeed;
	}
}

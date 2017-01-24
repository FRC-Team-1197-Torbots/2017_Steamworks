package org.usfirst.frc.team1197.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum TorCAN
{
	INSTANCE;
	
	private final CANTalon m_Rtalon1;
	private final CANTalon m_Rtalon2;
	private final CANTalon m_Rtalon3;
	private final CANTalon m_Ltalon1;
	private final CANTalon m_Ltalon2;
	private final CANTalon m_Ltalon3;
	
	private final AHRS gyro;
	private final double encoderTicksPerMeter = 8945.0; // (units: ticks per meter)
	private final double approximateSensorSpeed = 2368.51; // measured maximum (units: RPM)
	private final double quadEncNativeUnits = 512.0; // (units: ticks per revolution)
//	private final double kF = (1023.0) / ((approximateSensorSpeed * quadEncNativeUnits) / (600.0));
	private final double kF = 0.219791;
	private final double kP = 1.125; //1.125
	private final double kI = 0.0; //0.0
	private final double kD = 70.0; //70.0
	// absoluteMaxSpeed is in meters per second. Right now it comes out to about 4.405 m/s
	private final double absoluteMaxSpeed = (approximateSensorSpeed*quadEncNativeUnits)/(60*encoderTicksPerMeter);
	private final double trackWidth = 0.5786; // (units: meters (14.5 in inches))
	private final double halfTrackWidth = trackWidth / 2.0; // (units: meters)
	
	private final double backlash = 0.015; // (units: meters)
	
	private TorCAN(){
		gyro = new AHRS(SerialPort.Port.kMXP);
		
		m_Ltalon1 = new CANTalon(1);
		m_Ltalon2 = new CANTalon(2);
		m_Ltalon3 = new CANTalon(3);
		m_Rtalon1 = new CANTalon(4);
		m_Rtalon2 = new CANTalon(5);
		m_Rtalon3 = new CANTalon(6);
		
		m_Rtalon2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		m_Rtalon2.reverseSensor(false);
		m_Rtalon2.reverseOutput(false);
//		m_Rtalon2.setAllowableClosedLoopErr(10);
		m_Rtalon2.configNominalOutputVoltage(+0.0f, -0.0f);
		m_Rtalon2.configPeakOutputVoltage(+12.0f, -12.0f);
		m_Rtalon2.setProfile(0);
		m_Rtalon2.setF(kF); 
		m_Rtalon2.setP(kP); 
		m_Rtalon2.setI(kI); 
		m_Rtalon2.setD(kD);
		
		m_Rtalon1.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon1.set(m_Rtalon2.getDeviceID());
		m_Rtalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Rtalon3.set(m_Rtalon2.getDeviceID());
		
		m_Ltalon2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		m_Ltalon2.reverseSensor(true);
		m_Ltalon2.reverseOutput(true);
//		m_Ltalon2.setAllowableClosedLoopErr(10);
		m_Ltalon2.configNominalOutputVoltage(+0.0f, -0.0f);
		m_Ltalon2.configPeakOutputVoltage(+12.0f, -12.0f);
		m_Ltalon2.setProfile(0);
		m_Ltalon2.setF(kF);  
		m_Ltalon2.setP(kP); 
		m_Ltalon2.setI(kI); 
		m_Ltalon2.setD(kD); 
		
		m_Ltalon1.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon1.set(m_Ltalon2.getDeviceID());
		m_Ltalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		m_Ltalon3.set(m_Ltalon2.getDeviceID());

		m_Rtalon2.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
		m_Ltalon2.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 2);
	}

	public void SetDrive(double leftSpeed, double rightSpeed)
	{
		SetLeft(-leftSpeed);
		SetRight(rightSpeed);
	}
	
	//Setting the left master Talon's speed to the given parameter
	public void SetLeft(double speed)
	{
		m_Ltalon2.set(speed);
	}

	//Setting the right master Talon's speed to the given parameter
	public void SetRight(double speed)
	{
		m_Rtalon2.set(speed);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to percentVbus, so we can use it when the robot is in low gear. */
	public void choosePercentVbus(){
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	/* A method to change the right and left master Talon's control mode
	   to speed, so we can use it when the robot is in high gear. */
	public void chooseVelocityControl(){
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.Speed);
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.Speed);
	}
	
	public void chooseMotionProfileControl(){
		m_Rtalon2.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
		m_Ltalon2.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
	}
	
	public double getPosition(){
		SmartDashboard.putNumber("m_Rtalon2.getPosition()", m_Rtalon2.getPosition());
		SmartDashboard.putNumber("m_Ltalon2.getPosition()", m_Ltalon2.getPosition());
		return (m_Rtalon2.getPosition() + m_Ltalon2.getPosition()) * 0.5 / encoderTicksPerMeter; // (units: meters)
	}
	public double getVelocity(){
		return (m_Rtalon2.getSpeed() + m_Ltalon2.getSpeed()) * 0.5 * 10 / encoderTicksPerMeter; // (units: meters)
	}
	
	public double getHeading(){
		return (gyro.getAngle() * (Math.PI / 180)); // (units: radians)
	}
	public double getOmega(){
		return (gyro.getRate()); // (units: radians (contrary to navX documentation))
	}
	
	//1.555 is the conversion factor that we found experimentally.
	public void setTargets(double v, double omega){ 
		m_Rtalon2.set((v - omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);
		m_Ltalon2.set((v + omega * halfTrackWidth) * 0.1 * encoderTicksPerMeter);		
	}
	
	public void resetEncoder(){
		m_Rtalon2.setPosition(0);
		m_Ltalon2.setPosition(0);
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

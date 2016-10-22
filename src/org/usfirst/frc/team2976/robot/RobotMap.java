package org.usfirst.frc.team2976.robot;
/**
 * @author NeilHazra
 * 
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
	//Drive Motors
	public static final int RightFrontDriveMotor = 3; 
	public static final int LeftFrontDriveMotor = 2; 
	public static final int RightBackDriveMotor = 4; 
	public static final int LeftBackDriveMotor = 1;
	
	//Arm Motors
	public static final int RightArmMotor = 6; 
	public static final int LeftArmMotor = 5;
	
	//Encoders for Arm PID
	public static final int HallEffectSensor = 9;
	public static final int RightArmEncoderA = 3;
	public static final int RightArmEncoderB = 4;
	public static final int LeftArmEncoderA = 7;
	public static final int LeftArmEncoderB = 8;
	
	//Roller
	public static final int rollerMotorID = 7; 
	
	//Gyro for Drive Straight
	public static final int Gyro = 1;
	
	//Encoders for drive straight
	public static final int RightDriveEncoderA = 3; //FIXME
	public static final int RightDriveEncoderB = 4; //FIXME
	public static final int LeftDriveEncoderA = 7; 
	public static final int LeftDriveEncoderB = 8; 
}

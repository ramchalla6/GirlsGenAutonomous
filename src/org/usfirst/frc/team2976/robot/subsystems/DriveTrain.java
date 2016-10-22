package org.usfirst.frc.team2976.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive;
import util.PIDMain;
import util.PIDSource;

import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
/**
 *
 */
public class DriveTrain extends Subsystem {
    private SpeedController rightFrontMotor, leftFrontMotor;
    private SpeedController rightBackMotor, leftBackMotor;
    //private Encoder rightDriveEncoder, leftDriveEncoder;
	private AnalogGyro gyro;
	public PIDSource gyroPIDSource;
    public PIDMain DriveStraightPID;
    
    public RobotDrive m_drive;
    
	public DriveTrain()	{
		double gyro_kp = 0;
		double gyro_ki = 0;
		double gyro_kd = 0;
		
    	rightFrontMotor = new CANTalon(RobotMap.RightFrontDriveMotor);
    	leftFrontMotor = new CANTalon(RobotMap.LeftFrontDriveMotor);
    	rightBackMotor = new CANTalon(RobotMap.RightBackDriveMotor);
    	leftBackMotor = new CANTalon(RobotMap.LeftBackDriveMotor);
    	
    	rightFrontMotor.setInverted(true);
    	leftFrontMotor.setInverted(true);
    	rightBackMotor.setInverted(true);
    	leftBackMotor.setInverted(true);
    	
    	m_drive =  new RobotDrive(leftBackMotor, leftFrontMotor,rightBackMotor, rightFrontMotor); //Robot Drive Class
    	
    	//rightDriveEncoder = new Encoder(RobotMap.RightDriveEncoderA,RobotMap.RightDriveEncoderB);
    	//leftDriveEncoder = new Encoder(RobotMap.LeftDriveEncoderA,RobotMap.LeftArmEncoderB);
    	
    	gyro = new AnalogGyro(RobotMap.Gyro);
    
    	gyroPIDSource = new PIDSource() {
			public double getInput() {
				SmartDashboard.putNumber("RobotHeading", gyro.getAngle()%360);
				return gyro.getAngle();
			}
		};
		DriveStraightPID = new PIDMain(gyroPIDSource, 0, 100, gyro_kp, gyro_ki, gyro_kd);
	}
    public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoystick());
    }
    public double getGyro()	{
    	return DriveStraightPID.getInput();
    }
    public void arcadeDrive(double x, double y){
    	m_drive.arcadeDrive(y, x);
    }
    public void drive(double rightPower, double leftPower)	{
    	DriveStraightPID.isEnabled(false);
    	DriveStraightPID.resetPID();
    	_drive(rightPower,leftPower);
    }
    public void driveStraight(double currentHeading, double speed)	{
    	DriveStraightPID.isEnabled(true);
    	DriveStraightPID.setSetpoint(currentHeading);
    	double delta = DriveStraightPID.getOutput();

		SmartDashboard.putNumber("Left", -delta);
		SmartDashboard.putNumber("Right", delta);
		
    	_drive(speed+delta,speed-delta);
    }
    private void _drive(double right, double left)	{
    	m_drive.tankDrive(left, right);
    }
}


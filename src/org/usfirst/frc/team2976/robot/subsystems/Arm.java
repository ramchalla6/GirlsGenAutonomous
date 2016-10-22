package org.usfirst.frc.team2976.robot.subsystems;

import org.usfirst.frc.team2976.robot.OI;
import org.usfirst.frc.team2976.robot.Robot;
import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.ArmControl;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.PIDMain;
import util.PIDSource;

/**
 *
 */
public class Arm extends Subsystem {
	private CANTalon rightArm;
	private CANTalon leftArm;
	
	public Encoder rightArmEncoder, leftArmEncoder;//changed to public

	public PIDSource rightArmPIDSource;
	public PIDSource leftArmPIDSource;
	//public DigitalInput hallEffectSensor;
	public PIDMain rightArmPID;
	public PIDMain leftArmPID;

	private double kp = 0.007;
	private double ki = 0;
	private double kd = 0.0001;

	public boolean overAmped = false;
	public final double maxCurrent = 5; //TODO: this is a rough estimate

	public Arm() {
		rightArm = new CANTalon(RobotMap.RightArmMotor);
		leftArm = new CANTalon(RobotMap.LeftArmMotor);
		leftArmEncoder = new Encoder(RobotMap.LeftArmEncoderA, RobotMap.LeftArmEncoderB);
		rightArmEncoder = new Encoder(RobotMap.RightArmEncoderA, RobotMap.RightArmEncoderB);
		//hallEffectSensor = new DigitalInput(RobotMap.HallEffectSensor);
		rightArmPIDSource = new PIDSource()	{
			public double getInput() {
				SmartDashboard.putNumber("RightArm", rightArmEncoder.get());
				return rightArmEncoder.get();
			}
		};
		leftArmPIDSource = new PIDSource() {
			public double getInput() {
				SmartDashboard.putNumber("LeftArm", leftArmEncoder.get());
				return leftArmEncoder.get();
			}
		};
		rightArmPID = new PIDMain(rightArmPIDSource, 0, 100, kp, ki, kd);
		leftArmPID = new PIDMain(leftArmPIDSource, 0, 100, kp, ki, kd);
		rightArmPID.isEnabled(true);
		leftArmPID.isEnabled(true);
		
		rightArmPID.setOutputLimits(-1, 1);
		leftArmPID.setOutputLimits(-1, 1);
	}

	public void setPosition(double position) {
		rightArmPID.setSetpoint(position);
		leftArmPID.setSetpoint(position);
		//setPower(leftArmPID.getOutput(), leftArmPID.getOutput());
		setPower(Robot.oi.armStick.getRawAxis(OI.Axis.RY.getAxisNumber()), Robot.oi.armStick.getRawAxis(OI.Axis.RY.getAxisNumber()));
	}
	//public boolean getLimitSwitch()	{
		//return hallEffectSensor.get();
	//}
	public void setPower(double rightPower, double leftPower) {
		SmartDashboard.putNumber("ArmOutputCurrentAverage",
				(leftArm.getOutputCurrent() + rightArm.getOutputCurrent()) / 2);
		
		if (rightArm.getOutputCurrent() > maxCurrent || leftArm.getOutputVoltage() > maxCurrent) {
			overAmped = true;
		}
		if (Robot.oi.armStick.getPOV() != -1) {
			overAmped = false;
		}
		if (!overAmped) {
			rightArm.set(rightPower / 3);
			leftArm.set(-leftPower / 3);
		} else {
			rightArm.set(0);
			leftArm.set(0);
		}
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ArmControl());
	}
}
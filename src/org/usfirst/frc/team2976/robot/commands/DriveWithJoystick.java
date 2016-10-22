package org.usfirst.frc.team2976.robot.commands;

import org.usfirst.frc.team2976.robot.OI;
import org.usfirst.frc.team2976.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithJoystick extends Command {
	double threshold = 0.05;
	double currentHeading = 0;
	
    public DriveWithJoystick() {
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double x = Robot.oi.driveStick.getRawAxis(OI.Axis.RX.getAxisNumber());
    	double y = Robot.oi.driveStick.getRawAxis(OI.Axis.LY.getAxisNumber());
    	boolean driveStraight = Robot.oi.driveStick.getRawButton(OI.Button.X.getBtnNumber());
    
    	if(!driveStraight){
    		Robot.driveTrain.arcadeDrive(x, y);
    		currentHeading = Robot.driveTrain.getGyro();
    	}	else	{
    		Robot.driveTrain.driveStraight(currentHeading,y);
    	}
    }
    	
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }
    
    // Called once after isFinished returns true
    protected void end() {
    }
    	
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

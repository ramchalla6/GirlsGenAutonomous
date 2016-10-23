package org.usfirst.frc.team2976.robot.commands;
import org.usfirst.frc.team2976.robot.OI;
import org.usfirst.frc.team2976.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import util.PIDMain;
/**
 *@author NeilHazra
 */
public class ArmControl extends Command {
	double ArmDownValue = -550;
    public ArmControl() {
    	requires(Robot.arm);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double position = -Robot.oi.armStick.getRawAxis(OI.Axis.RY.getAxisNumber());
    	position = PIDMain.map(position, -1, 1, ArmDownValue, 0);
    	/*if(Robot.arm.getLimitSwitch()){ //if halleffectsensor is pressed
    		Robot.arm.leftArmEncoder.reset();
    		Robot.arm.rightArmEncoder.reset();
    	}*/
    	
    	if(Math.abs(Robot.oi.armStick.getRawAxis(OI.Axis.RY.getAxisNumber()))<0.1)	{
    		Robot.arm.setPower(0, 0);
    		Robot.arm.leftArmPID.isEnabled(false);
    		Robot.arm.rightArmPID.isEnabled(false);
    	}	else	{
    		Robot.arm.leftArmPID.isEnabled(true);
    		Robot.arm.rightArmPID.isEnabled(true);
    		Robot.arm.setPosition(position);
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

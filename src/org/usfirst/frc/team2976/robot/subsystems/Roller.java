package org.usfirst.frc.team2976.robot.subsystems;

import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.RollerForward;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Roller extends Subsystem {
    private CANTalon roller;
	
    public Roller()	{
		roller = new CANTalon(RobotMap.rollerMotorID);
	}	
	public void setRollerPower(double power)	{
		roller.set(power);
	}
    public void initDefaultCommand() {
        setDefaultCommand(new RollerForward(0));
    }
}


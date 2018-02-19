package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousDefault extends CommandGroup {

	public AutonomousDefault() {
		// Drive forward into a wall or whatever
		addSequential(new DriveStraight(14));
	}
}

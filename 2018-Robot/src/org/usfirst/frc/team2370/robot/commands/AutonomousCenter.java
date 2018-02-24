package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCenter extends CommandGroup {

	public AutonomousCenter() {
		switch (Robot.fieldMessage) {
		case "lll":
			addSequential(new DriveLeft(), 2.5);
			addSequential(new DriveRight(), 2);
			addParallel(new PushElevatorSol());
			break;
		case "rrr":
			addSequential(new DriveRight(), 2.5);
			addSequential(new DriveLeft(), 2);
			addParallel(new PushElevatorSol());
			break;
		case "rlr":
			addSequential(new DriveRight(), 2.5);
			addSequential(new DriveLeft(), 2);
			addParallel(new PushElevatorSol());
			break;
		case "lrl":
			addSequential(new DriveLeft(), 2.5);
			addSequential(new DriveRight(), 2);
			addParallel(new PushElevatorSol());
			break;
		}
	}
}

package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousLeft extends CommandGroup {

	public AutonomousLeft() {
		switch (RobotMap.fieldMessage) {
		case "lll":
			addSequential(new DriveStraight(142), 3.5);
			addSequential(new TurnRight(90), 2);
			addSequential(new SetElevatorPosition(-3000));
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(90), 2.5);
			addSequential(new DriveStraight(5) , 2);
		case "rrr":
			addSequential(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addParallel(new PullGripper());
			break;
		case "rlr":
			addSequential(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addParallel(new PullGripper());
			break;
		case "lrl":
			addSequential(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addParallel(new PullGripper());
			break;
		}
	}
}

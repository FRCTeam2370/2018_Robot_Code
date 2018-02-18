package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRight extends CommandGroup {

	public AutonomousRight() {

		switch (RobotMap.fieldMessage) {
		case "lll":

			// Drive Forwadr Beyond Switch
			addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addSequential(new PullGripper());

			break;

		case "rrr":
			addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addSequential(new PullGripper());

			break;

		case "lrl":
			addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addSequential(new PullGripper());

			break;

		case "rlr":
			addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(0));
			addSequential(new PullGripper());

			break;

		}

	}

}

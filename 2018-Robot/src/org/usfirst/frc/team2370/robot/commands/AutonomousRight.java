package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRight extends CommandGroup {

	public AutonomousRight() {

		switch (RobotMap.fieldMessage) {
		case "lll":

			// Drive past line
			// addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(14));
			// Stop

			break;

		case "rrr":

			// Place powercube on scale in front of the robot.
			// addParallel(new ElevatorToBottom());
			addParallel(new SetElevatorPosition(-2000));
			addParallel(new DriveStraight(139));
			addSequential(new TurnLeft(90));
			addSequential(new DriveStraight(5));
			addSequential(new PushGripper());
			addSequential(new DriveStraight());
			addSequential(new TurnRight(90));
			// stop

			break;

		case "lrl":
			// Drive to the scale and place the cube
			addParallel(new ElevatorToBottom());
			addParallel(new DriveStraight(30));
			addSequential(new TurnLeft(90));
			addSequential(new PushGripper());
			addSequential(new TurnRight(90));

			break;

		case "rlr": 
			// Place powercube on scale in front of the robot.
			// addParallel(new ElevatorToBottom());
			addParallel(new SetElevatorPosition(10));
			addParallel(new DriveStraight(15));
			addSequential(new TurnLeft(90));
			addSequential(new PushGripper());
			addSequential(new TurnRight(90));
			// stop

		}

	}

}

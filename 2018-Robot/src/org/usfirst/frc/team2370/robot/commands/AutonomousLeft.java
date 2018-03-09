package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousLeft extends CommandGroup {

	public AutonomousLeft() {
		switch (Robot.fieldMessage) {
		case "lll":
			// Place the cube on the switch if you are on the right.
			addSequential(new DriveStraight(142), 3.25);
			addParallel(new PushElevatorSol(), 0.1);
			// addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnRight(86), 1.5);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			// addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(86), 2.5);
			addSequential(new DriveStraight(5), 2);
			break;
		case "rrr":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(220), 4.3);
			addSequential(new TurnRight(90), 1.5);
			addSequential(new DriveStraight(142), 3.4);
			addSequential(new TurnRight(65), 1);
			addParallel(new PushElevatorSol(), .1);
			// addSequential(new SetElevatorPosition(-3000));
			addParallel(new DriveStraight(10), 0.8);
			addParallel(new PushGripper());
			break;

		case "rlr":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(220), 4.3);
			addSequential(new TurnRight(90), 1.5);
			addSequential(new DriveStraight(142), 3.4);
			addSequential(new TurnRight(65), 1);
			addParallel(new PushElevatorSol(), .1);
			// addSequential(new SetElevatorPosition(-3000));
			addParallel(new DriveStraight(10), 0.8);
			addParallel(new PushGripper());
			break;

		case "lrl":
			// Place the cube on the switch if you are on the right.
			addSequential(new DriveStraight(142), 3.25);
			addParallel(new PushElevatorSol(), 0.1);
			// addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnRight(86), 1.5);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			// addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(86), 2.5);
			addSequential(new DriveStraight(5), 2);
			break;
		default:
			addSequential(new DriveStraight(150), 5);
			break;
		}
	}
}

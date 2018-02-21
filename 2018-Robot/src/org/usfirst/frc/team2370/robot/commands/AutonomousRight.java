package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRight extends CommandGroup {

	public AutonomousRight() {

		switch (RobotMap.fieldMessage) {
		case "lll":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(220), 4.3);
			addSequential(new TurnLeft(90), 1);
			addSequential(new DriveStraight(137), 3.1);
			addSequential(new TurnLeft(65), 1);
			addParallel(new PushElevatorSol(), .1);
			//addSequential(new SetElevatorPosition(-3000));
			addParallel(new DriveStraight(10), 0.8);
			addParallel(new PushGripper());
			break;
		case "rrr":
			// Place the cube on the switch if you are on the right.
			addSequential(new DriveStraight(142), 3.5);
			addParallel(new PushElevatorSol(), 0.1);
			//addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnLeft(90), 2);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			//addSequential(new ElevatorToBottom());
			addSequential(new TurnRight(90), 2.5);
			addSequential(new DriveStraight(5), 2);
			break;
		// stop
		case "lrl":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(214), 5);
			addSequential(new TurnLeft(80), 1.5);
			addSequential(new DriveStraight(137), 4);
			addSequential(new TurnLeft(7), 1);
			addSequential(new PushElevatorSol(), .1);
			//addSequential(new SetElevatorPosition(-3000));
			addSequential(new DriveStraight(10), 1);
			addSequential(new PushGripper());
			break;
		case "rlr":
			// start
			addSequential(new DriveStraight(142), 3.5);
			addParallel(new PushElevatorSol(), 0.1);
			addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnLeft(90), 2);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			addSequential(new ElevatorToBottom());
			addSequential(new TurnRight(90), 2.5);
			addSequential(new DriveStraight(5), 2);
			break;
		// stop
		}

	}

}

package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRight extends CommandGroup {

	public AutonomousRight() {

		switch (Robot.fieldMessage) {

		case "lll":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(220), 4.3);
			addSequential(new TurnLeft(90), 1.5);
			addSequential(new DriveStraight(142), 3.4);
			addSequential(new TurnLeft(65), 1);
			addParallel(new PushElevatorSol(), .1);
			// addSequential(new SetElevatorPosition(-3000));
			addParallel(new DriveStraight(10), 0.8);
			addParallel(new PushGripper());
			break;
		case "rrr":

			addParallel(new PushElevatorSol());
			addSequential(new DriveStraight(241), 4);
			
			addParallel(new ElevatorToTop());
			addSequential(new TurnLeft(45), 2.5);
			addSequential(new DriveForwardsSlow(9), 2);
			addSequential(new PushGripper(), 2);
			addSequential(new DriveBackwardsSlow(8), 2);
			addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(75), 2.25);
			
			addSequential(new DriveStraight(28), 2);
			break;
		// stop
		case "lrl":

			addParallel(new PushElevatorSol());
			addSequential(new DriveStraight(241), 4);
			
			addParallel(new ElevatorToTop());
			addSequential(new TurnLeft(45), 2.5);
			addSequential(new DriveForwardsSlow(9), 2);
			addSequential(new PushGripper(), 2);
			addSequential(new DriveBackwardsSlow(8), 2);
			addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(75), 2.25);
			
			addSequential(new DriveStraight(28), 2);
			break;
		case "rlr":
			// start
			addSequential(new DriveStraight(142), 3.25);
			addParallel(new PushElevatorSol(), 0.1);
			//addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnLeft(86), 1.5);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			addSequential(new ElevatorToBottom());
			addSequential(new TurnRight(86), 2.5);
			addSequential(new DriveStraight(5), 2);
			
			
			
			break;
		// stop
		default:
			addSequential(new DriveStraight(300), 15);
			break;
		}

	}

}
